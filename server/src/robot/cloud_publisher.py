import asyncio
import json
import logging
import os
from typing import Iterable, List, Optional

from aiohttp import ClientSession
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer, MediaRecorder

from camera import MultiCameraVideoTrack
from robot.controller import Controller

logger = logging.getLogger(__name__)


def _get_env_audio_spec(kind: str) -> tuple[Optional[str], Optional[str]]:
    dev = os.getenv(f"ROBOT_AUDIO_{kind.upper()}_DEVICE")
    fmt = os.getenv(f"ROBOT_AUDIO_{kind.upper()}_FORMAT")
    return dev, fmt


def _open_media_player() -> Optional[MediaPlayer]:
    dev, fmt = _get_env_audio_spec("input")
    attempts: list[tuple[str, str, str]] = []
    if dev and fmt:
        attempts.append((dev, fmt, "env override"))
    attempts += [("default", "pulse", "PulseAudio default"), ("default", "alsa", "ALSA default")]

    for device, fmt_val, label in attempts:
        try:
            player = MediaPlayer(device, format=fmt_val)
            if player.audio:
                logger.info("Audio input ready via %s (%s,%s)", label, device, fmt_val)
                return player
        except Exception as exc:
            logger.debug("Audio input %s unavailable (%s,%s): %s", label, device, fmt_val, exc)
    logger.warning("Audio input unavailable; set ROBOT_AUDIO_INPUT_DEVICE/FORMAT to target a specific device.")
    return None


def _open_media_recorder(track) -> Optional[MediaRecorder]:
    dev, fmt = _get_env_audio_spec("output")
    attempts: list[tuple[str, str, str]] = []
    if dev and fmt:
        attempts.append((dev, fmt, "env override"))
    attempts += [("default", "pulse", "PulseAudio default"), ("default", "alsa", "ALSA default")]

    for device, fmt_val, label in attempts:
        try:
            recorder = MediaRecorder(device, format=fmt_val)
            recorder.addTrack(track)
            logger.info("Audio output ready via %s (%s,%s)", label, device, fmt_val)
            return recorder
        except Exception as exc:
            logger.debug("Audio output %s unavailable (%s,%s): %s", label, device, fmt_val, exc)
    logger.warning("Audio output unavailable; set ROBOT_AUDIO_OUTPUT_DEVICE/FORMAT to target a specific device.")
    return None


def _parse_camera_ids(env_value: Optional[str], default: Iterable[int]) -> List[int]:
    if not env_value:
        return list(default)
    parts = [p.strip() for p in env_value.split(",") if p.strip()]
    ids: List[int] = []
    for p in parts:
        try:
            ids.append(int(p))
        except ValueError:
            logger.warning("Ignoring non-integer camera id: %s", p)
    return ids or list(default)


def _handle_control_message(msg: str, controller: Controller):
    try:
        data = json.loads(msg)
    except Exception as exc:
        logger.debug("Control message not JSON: %s", exc)
        return

    if "head" in data:
        head = data["head"] or {}
        try:
            pan = float(head.get("pan", 0.0))
            tilt = float(head.get("tilt", 0.0))
            controller.move_head(pan_deg=pan, tilt_deg=tilt)
        except Exception as exc:
            logger.warning("Failed to apply head command: %s", exc)

    if "base" in data:
        base = data["base"] or {}
        try:
            x = float(base.get("x", 0.0))
            y = float(base.get("y", 0.0))
            theta = float(base.get("theta", 0.0))
            controller.move_base(x_vel=x, y_vel=y, theta_vel=theta)
        except Exception as exc:
            logger.warning("Failed to apply base command: %s", exc)


async def run_publisher(cloud_url: str, camera_ids: Iterable[int]):
    controller = Controller()
    pc = RTCPeerConnection()

    control_dc = pc.createDataChannel("control")

    @control_dc.on("open")
    def _on_open():
        logger.info("Control data channel opened")

    @control_dc.on("message")
    def _on_message(msg):
        _handle_control_message(msg, controller)

    # Video tracks
    pc.addTrack(MultiCameraVideoTrack(camera_ids=list(camera_ids)))

    # Optional audio capture
    player = _open_media_player()
    if player and player.audio:
        pc.addTrack(player.audio)

    @pc.on("track")
    async def on_track(track):
        if track.kind == "audio":
            recorder = _open_media_recorder(track)
            if recorder:
                try:
                    await recorder.start()
                except Exception as exc:
                    logger.debug("Failed to start audio recorder: %s", exc)

    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)

    async with ClientSession() as session:
        async with session.post(f"{cloud_url.rstrip('/')}/robot_offer", json={"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}) as resp:
            if resp.status != 200:
                text = await resp.text()
                raise RuntimeError(f"robot_offer failed: {resp.status} {text}")
            ans = await resp.json()

    await pc.setRemoteDescription(RTCSessionDescription(sdp=ans["sdp"], type=ans["type"]))
    logger.info("Robot connected to cloud server; awaiting clients and control messages")

    # Keep running
    await asyncio.Future()


def main():
    logging.basicConfig(level=logging.INFO)
    cloud_url = os.environ.get("CLOUD_URL", "http://localhost:8080")
    camera_env = os.environ.get("ROBOT_CAMERA_IDS")
    camera_ids = _parse_camera_ids(camera_env, default=(0, 2, 4))
    try:
        asyncio.run(run_publisher(cloud_url=cloud_url, camera_ids=camera_ids))
    except KeyboardInterrupt:
        logger.info("Shutting down robot publisher")


if __name__ == "__main__":
    main()
