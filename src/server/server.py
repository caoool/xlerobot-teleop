import asyncio
import json
import logging
import os
from pathlib import Path
from typing import Iterable, List, Optional

from aiohttp import web
from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    MediaStreamTrack,
    RTCConfiguration,
    RTCIceServer,
)
from aiortc.contrib.media import MediaRelay

logger = logging.getLogger(__name__)

# Global state
robot_pc: Optional[RTCPeerConnection] = None
user_pc: Optional[RTCPeerConnection] = None

robot_video_track: Optional[MediaStreamTrack] = None
robot_audio_track: Optional[MediaStreamTrack] = None
robot_control_channel = None
relay = MediaRelay()

DEFAULT_ICE_SERVERS = ["stun:stun.l.google.com:19302"]


def _get_ice_configuration() -> RTCConfiguration:
    env_val = os.getenv("ICE_SERVERS")
    servers: list[str] = DEFAULT_ICE_SERVERS
    if env_val:
        try:
            parsed = json.loads(env_val)
            # Accept either list of strings or list of {"urls": [...]}
            if isinstance(parsed, list) and parsed:
                if isinstance(parsed[0], str):
                    servers = [str(u) for u in parsed]
                elif isinstance(parsed[0], dict) and "urls" in parsed[0]:
                    servers = (
                        parsed[0]["urls"]
                        if isinstance(parsed[0]["urls"], list)
                        else [parsed[0]["urls"]]
                    )
        except Exception:
            logger.warning("Invalid ICE_SERVERS env value; using default STUN")
    ice_servers = [RTCIceServer(urls=servers)]
    return RTCConfiguration(iceServers=ice_servers)


STATIC_DIR = Path(__file__).parent / "static"
INDEX_FILE = STATIC_DIR / "index.html"


async def on_shutdown(app: web.Application):
    global robot_pc, user_pc
    if robot_pc:
        await robot_pc.close()
    if user_pc:
        await user_pc.close()


async def handle_robot_offer(request: web.Request) -> web.Response:
    global robot_pc, robot_video_track, robot_audio_track, robot_control_channel, relay

    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    if robot_pc:
        await robot_pc.close()

    pc = RTCPeerConnection(configuration=_get_ice_configuration())
    robot_pc = pc

    logger.info("Robot connecting...")

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        logger.info("Robot connection state is %s", pc.connectionState)
        if pc.connectionState in {"failed", "closed"}:
            global robot_pc, robot_video_track, robot_audio_track, robot_control_channel, relay
            if robot_pc == pc:
                robot_pc = None
                robot_video_track = None
                robot_audio_track = None
                robot_control_channel = None
                relay = MediaRelay()

    @pc.on("track")
    def on_track(track):
        global robot_video_track, robot_audio_track
        logger.info(f"Robot track received: {track.kind}")
        if track.kind == "video":
            robot_video_track = track
        elif track.kind == "audio":
            robot_audio_track = track

    @pc.on("datachannel")
    def on_datachannel(channel):
        global robot_control_channel
        logger.info(f"Robot data channel received: {channel.label}")
        if channel.label == "control":
            robot_control_channel = channel

    await pc.setRemoteDescription(offer)

    # Ensure we answer with sendrecv for audio so we can send user audio later
    for t in pc.getTransceivers():
        if t.kind == "audio":
            t.direction = "sendrecv"

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {
                "sdp": pc.localDescription.sdp,
                "type": pc.localDescription.type,
            }
        ),
    )


async def handle_user_offer(request: web.Request) -> web.Response:
    global user_pc, robot_video_track, robot_audio_track, robot_pc, relay

    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    if user_pc:
        await user_pc.close()

    pc = RTCPeerConnection(configuration=_get_ice_configuration())
    user_pc = pc

    logger.info("User connecting...")

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        logger.info("User connection state is %s", pc.connectionState)
        if pc.connectionState in {"failed", "closed"}:
            global user_pc
            if user_pc == pc:
                user_pc = None

    # Add Robot Tracks to User PC
    if robot_video_track:
        pc.addTrack(relay.subscribe(robot_video_track))
        logger.info("Added robot video track to user")

    if robot_audio_track:
        pc.addTrack(relay.subscribe(robot_audio_track))
        logger.info("Added robot audio track to user")

    # Handle User Audio (to send to Robot)
    @pc.on("track")
    def on_track(track):
        if track.kind == "audio" and robot_pc:
            logger.info("Received user audio track, forwarding to robot")
            # Find the sender on robot_pc that sends audio
            for sender in robot_pc.getSenders():
                if sender.kind == "audio":
                    sender.replaceTrack(track)
                    break

    # Handle User Control (Data Channel)
    @pc.on("datachannel")
    def on_datachannel(channel):
        if channel.label == "control":
            logger.info("User control channel established")

            @channel.on("message")
            def on_message(message):
                if robot_control_channel and robot_control_channel.readyState == "open":
                    robot_control_channel.send(message)

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {
                "sdp": pc.localDescription.sdp,
                "type": pc.localDescription.type,
            }
        ),
    )


def index(request):
    return web.FileResponse(INDEX_FILE)


def create_app() -> web.Application:
    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_post("/offer", handle_user_offer)
    app.router.add_post("/robot/offer", handle_robot_offer)
    app.on_shutdown.append(on_shutdown)
    return app


def run_server(host: str = "0.0.0.0", port: int = 8080):
    logging.basicConfig(level=logging.INFO)
    logger.info("Starting Cloud Server...")
    web.run_app(create_app(), host=host, port=port)


if __name__ == "__main__":
    run_server()
