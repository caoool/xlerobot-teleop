import os
import sys

# Add src directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import asyncio
import json
import logging
import pathlib
from typing import Optional, Tuple

import aiohttp
from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    RTCConfiguration,
    RTCIceServer,
)
from aiortc.contrib.media import MediaPlayer, MediaRecorder

from camera import MultiCameraVideoTrack
from robot.controller import Controller

logger = logging.getLogger(__name__)

DISABLE_AUDIO = os.getenv("ROBOT_DISABLE_AUDIO") == "1"
AUDIO_LATENCY_MS = int(os.getenv("ROBOT_AUDIO_LATENCY_MS", "20"))
AUDIO_PTIME_MS = int(os.getenv("ROBOT_AUDIO_PTIME_MS", "20"))
AUDIO_RATE = os.getenv("ROBOT_AUDIO_RATE", "8000")
AUDIO_CHANNELS = os.getenv("ROBOT_AUDIO_CHANNELS", "1")

controller: Controller | None = None


def _ensure_alsa_config_env():
    """Set ALSA_CONFIG_PATH to a common default if missing and present on disk."""
    if not os.getenv("ALSA_CONFIG_PATH"):
        default_path = pathlib.Path("/usr/share/alsa/alsa.conf")
        if default_path.exists():
            os.environ["ALSA_CONFIG_PATH"] = str(default_path)
            logger.debug("ALSA_CONFIG_PATH set to %s", default_path)


def _get_env_audio_spec(kind: str) -> Tuple[Optional[str], Optional[str]]:
    """Return (device, format) from env for given kind ('input'|'output')."""
    dev = os.getenv(f"ROBOT_AUDIO_{kind.upper()}_DEVICE")
    fmt = os.getenv(f"ROBOT_AUDIO_{kind.upper()}_FORMAT")

    # Default to plughw:2,0/alsa if not set (hardcoded fallback per user request)
    if not dev:
        dev = "plughw:2,0"
    if not fmt:
        fmt = "alsa"

    return dev, fmt


def _open_media_player(kind: str) -> Optional[MediaPlayer]:
    """Try to open a MediaPlayer for audio input.

    Order: env override -> pulse default -> alsa default. Logs only once on failure.
    """
    if DISABLE_AUDIO:
        logger.info("Audio input disabled via ROBOT_DISABLE_AUDIO=1")
        return None
    _ensure_alsa_config_env()
    env_dev, env_fmt = _get_env_audio_spec("input")
    attempts = []
    if env_dev and env_fmt:
        attempts.append((env_dev, env_fmt, "env override"))
    attempts += [
        ("default", "pulse", "PulseAudio default"),
        ("default", "alsa", "ALSA default"),
    ]

    options = {
        "audio_buffer_size": str(AUDIO_LATENCY_MS),
        "sample_rate": AUDIO_RATE,
        "channels": AUDIO_CHANNELS,
    }

    for device, fmt, label in attempts:
        try:
            player = MediaPlayer(device, format=fmt, options=options)
            if player.audio:
                logger.info("Audio input ready via %s (%s,%s)", label, device, fmt)
                return player
        except Exception as exc:
            logger.debug(
                "Audio input %s unavailable (%s,%s): %s", label, device, fmt, exc
            )

    logger.warning("Audio input unavailable")
    return None


def _open_media_recorder(kind: str, track) -> Optional[MediaRecorder]:
    """Try to open MediaRecorder for audio output (play to speakers).

    Order: env override -> pulse default -> alsa default.
    """
    if DISABLE_AUDIO:
        logger.info("Audio output disabled via ROBOT_DISABLE_AUDIO=1")
        return None
    _ensure_alsa_config_env()
    env_dev, env_fmt = _get_env_audio_spec("output")
    attempts = []
    if env_dev and env_fmt:
        attempts.append((env_dev, env_fmt, "env override"))
    attempts += [
        ("default", "pulse", "PulseAudio default"),
        ("default", "alsa", "ALSA default"),
    ]

    for device, fmt, label in attempts:
        try:
            # MediaRecorder with ALSA doesn't use options like audio_buffer_size/sample_rate/channels
            # in the same way as input, and warns if they are unused.
            recorder = MediaRecorder(device, format=fmt)
            recorder.addTrack(track)
            logger.info("Audio output ready via %s (%s,%s)", label, device, fmt)
            return recorder
        except Exception as exc:
            logger.debug(
                "Audio output %s unavailable (%s,%s): %s", label, device, fmt, exc
            )

    logger.warning("Audio output unavailable")
    return None


async def run_robot_client(server_url: str, camera_ids: list[int]):
    global controller
    if controller is None:
        controller = Controller()
    retry_delay = 3

    ice_env = os.getenv("ROBOT_ICE_SERVERS")
    ice_urls: list[str] = [
        "turn:47.242.85.149:3478?transport=udp",
        "turn:47.242.85.149:3478?transport=tcp",
        "stun:47.242.85.149:3478",
    ]
    if ice_env:
        try:
            parsed = json.loads(ice_env)
            if isinstance(parsed, list) and parsed:
                if isinstance(parsed[0], str):
                    ice_urls = [str(u) for u in parsed]
                elif isinstance(parsed[0], dict):
                    collected: list[str] = []
                    for entry in parsed:
                        if not isinstance(entry, dict) or "urls" not in entry:
                            continue
                        urls = entry["urls"]
                        if isinstance(urls, list):
                            collected.extend([str(u) for u in urls])
                        else:
                            collected.append(str(urls))
                    if collected:
                        ice_urls = collected
        except Exception:
            logger.warning("Invalid ROBOT_ICE_SERVERS, using empty list")
    ice_servers = (
        [RTCIceServer(urls=ice_urls, username="lu", credential="880919Lu")]
        if ice_urls
        else []
    )
    ice_config = RTCConfiguration(iceServers=ice_servers)

    while True:
        pc = RTCPeerConnection(configuration=ice_config)
        video_track = None
        audio_player = None
        recorder = None

        # Add Video
        try:
            video_track = MultiCameraVideoTrack(camera_ids=camera_ids)
            pc.addTrack(video_track)
            logger.info("Added video track")
        except Exception as e:
            logger.error(f"Failed to add video track: {e}")

        # Add Audio (Mic)
        audio_player = _open_media_player("input")
        if audio_player and audio_player.audio:
            pc.addTrack(audio_player.audio)
            logger.info("Added audio track (mic)")
        else:
            pc.addTransceiver("audio", direction="recvonly")
            logger.info("Added audio transceiver (recvonly)")

        # Create Data Channel for control
        control_channel = pc.createDataChannel("control")

        @control_channel.on("open")
        def on_open():
            logger.info("Control channel open")

        @control_channel.on("message")
        def on_message(message):
            logger.info(f"Received control message: {message}")
            try:
                data = json.loads(message)
                if data.get("camera_settings"):
                    try:
                        video_track.apply_settings(data.get("camera_settings"))
                    except Exception as exc:
                        logger.error("Failed to apply camera settings: %s", exc)

                if data.get("layout_order"):
                    try:
                        video_track.set_layout(data.get("layout_order"))
                    except Exception as exc:
                        logger.error("Failed to set layout: %s", exc)
                elif data.get("cycle_layout"):
                    try:
                        video_track.cycle_layout()
                    except Exception as exc:
                        logger.error("Failed to cycle cameras: %s", exc)
                elif data.get("swap_cameras"):
                    try:
                        video_track.swap_primary_secondary()
                    except Exception as exc:
                        logger.error("Failed to swap cameras: %s", exc)
                elif data.get("single_camera") is not None:
                    try:
                        payload = data.get("single_camera") or {}
                        enabled = bool(payload.get("enabled"))
                        cam_index = payload.get("index")
                        if cam_index is not None:
                            cam_index = int(cam_index)
                        video_track.set_single_camera(enabled, cam_index)
                    except Exception as exc:
                        logger.error("Failed to set single-camera mode: %s", exc)
                elif "pan" in data and "tilt" in data:
                    controller.move_head(
                        pan_deg=float(data["pan"]), tilt_deg=float(data["tilt"])
                    )
                elif "x" in data:  # base move
                    controller.move_base(
                        x_vel=float(data.get("x", 0)),
                        theta_vel=float(data.get("theta", 0)),
                        y_vel=float(data.get("y", 0)),
                    )
            except Exception as e:
                logger.error(f"Error processing control message: {e}")

        # Handle incoming audio track (Speaker)
        @pc.on("track")
        def on_track(track):
            logger.info(f"Received track: {track.kind}")
            if track.kind == "audio":
                recorder = _open_media_recorder("output", track)
                if recorder:
                    asyncio.ensure_future(recorder.start())
                    logger.info("Started recording audio to speakers")

        # Create Offer
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        logger.info(f"Connecting to {server_url}...")

        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{server_url}/robot/offer",
                    json={
                        "sdp": pc.localDescription.sdp,
                        "type": pc.localDescription.type,
                    },
                ) as resp:
                    if resp.status != 200:
                        logger.error(
                            "Failed to connect to server: %s", await resp.text()
                        )
                        raise RuntimeError("Offer rejected")
                    answer = await resp.json()

            await pc.setRemoteDescription(
                RTCSessionDescription(sdp=answer["sdp"], type=answer["type"])
            )
            logger.info("Connected to Cloud Server")

            # Keep alive
            while True:
                await asyncio.sleep(1)
                if pc.connectionState in ["failed", "closed"]:
                    logger.error("Connection lost")
                    break
        except Exception as e:
            logger.error(f"Connection error: {e}")
        finally:
            try:
                await pc.close()
            except Exception:
                pass

            # Stop media resources to free camera/audio handles before retrying.
            try:
                if recorder:
                    await recorder.stop()
            except Exception:
                pass

            try:
                if video_track:
                    video_track.stop()
            except Exception:
                pass

            try:
                if audio_player and hasattr(audio_player, "stop"):
                    audio_player.stop()
            except Exception:
                pass

            logger.info("Retrying connection in %s seconds...", retry_delay)
            await asyncio.sleep(retry_delay)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    server_url = os.getenv("CLOUD_SERVER_URL", "http://localhost:8080")
    # Default camera IDs; override with ROBOT_CAMERA_IDS="0,2,4"
    env_ids = os.getenv("ROBOT_CAMERA_IDS")
    if env_ids:
        try:
            camera_ids = [int(x) for x in env_ids.split(",") if x.strip()]
        except ValueError:
            logger.warning(
                "Invalid ROBOT_CAMERA_IDS value '%s', falling back to defaults", env_ids
            )
            camera_ids = [0, 2, 4]
    else:
        camera_ids = [0, 2, 4]
    asyncio.run(run_robot_client(server_url, camera_ids))
