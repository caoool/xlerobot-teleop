import logging
import os
from typing import Optional, Set

from aiohttp import web
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaRelay

logger = logging.getLogger(__name__)

# One robot publisher at a time
robot_pc: Optional[RTCPeerConnection] = None
robot_tracks: list[MediaStreamTrack] = []
robot_control_channel = None

clients: Set[RTCPeerConnection] = set()
relay = MediaRelay()


async def _cleanup_pc(pc: RTCPeerConnection):
    try:
        await pc.close()
    except Exception:
        pass


def _attach_tracks_to_client(pc: RTCPeerConnection):
    for track in robot_tracks:
        try:
            pc.addTrack(relay.subscribe(track))
        except Exception as exc:
            logger.warning("Failed to add track to client: %s", exc)


async def robot_offer(request: web.Request) -> web.Response:
    global robot_pc, robot_tracks, robot_control_channel
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    # Replace existing robot connection
    if robot_pc:
        await _cleanup_pc(robot_pc)
        robot_pc = None
        robot_tracks = []
        robot_control_channel = None

    pc = RTCPeerConnection()
    robot_pc = pc

    @pc.on("connectionstatechange")
    async def on_state_change():
        if pc.connectionState in {"failed", "closed"}:
            await _cleanup_pc(pc)
            robot_tracks.clear()
            if robot_pc is pc:
                robot_pc = None
                robot_control_channel = None

    @pc.on("track")
    def on_track(track: MediaStreamTrack):
        logger.info("Robot track received: %s", track.kind)
        robot_tracks.append(track)
        # Attach to existing clients
        for cpc in list(clients):
            _attach_tracks_to_client(cpc)

    @pc.on("datachannel")
    def on_dc(channel):
        global robot_control_channel
        if channel.label == "control":
            robot_control_channel = channel
            logger.info("Robot control channel ready")

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.json_response({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})


async def client_offer(request: web.Request) -> web.Response:
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    clients.add(pc)

    @pc.on("connectionstatechange")
    async def on_state_change():
        if pc.connectionState in {"failed", "closed"}:
            await _cleanup_pc(pc)
            clients.discard(pc)

    @pc.on("datachannel")
    def on_dc(channel):
        if channel.label == "control":
            @channel.on("message")
            def on_message(msg):
                if robot_control_channel:
                    try:
                        robot_control_channel.send(msg)
                    except Exception as exc:
                        logger.warning("Failed to forward control message: %s", exc)

    # Attach current robot tracks if present
    _attach_tracks_to_client(pc)

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.json_response({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})


def create_app() -> web.Application:
    app = web.Application()
    app.router.add_post("/robot_offer", robot_offer)
    app.router.add_post("/client_offer", client_offer)
    return app


def run_server(host: str = "0.0.0.0", port: int = 8080):
    logger.info("Starting cloud signaling/relay server on %s:%s", host, port)
    web.run_app(create_app(), host=host, port=port)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    host = os.environ.get("HOST", "0.0.0.0")
    port_env = os.environ.get("PORT") or os.environ.get("CLOUD_SERVER_PORT")
    port = int(port_env) if port_env else 8080
    run_server(host=host, port=port)
