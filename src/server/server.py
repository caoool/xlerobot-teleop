import base64
import json
import logging
import os
import secrets
import ssl
from pathlib import Path
from typing import Optional

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

# Authentication configuration
AUTH_USERNAME = os.getenv("AUTH_USERNAME", "admin")
AUTH_PASSWORD = os.getenv("AUTH_PASSWORD", "teleop123")
AUTH_ENABLED = os.getenv("AUTH_ENABLED", "true").lower() == "true"

# Session tokens (in-memory store)
valid_sessions: set[str] = set()

# Low-latency tuning: prefer UDP, aggressive ICE, shorter timeouts
RTC_CONFIG_OPTIONS = {
    "iceCandidatePoolSize": 10,  # Pre-gather ICE candidates
    "bundlePolicy": "max-bundle",  # Multiplex all streams on single transport
    "rtcpMuxPolicy": "require",  # Reduce ports needed
}
# Silence noisy VP8 decoder warnings from aiortc when packets drop.
logging.getLogger("aiortc.codecs.vpx").setLevel(logging.ERROR)

DEFAULT_PORT = 8080

# Global state
robot_pc: Optional[RTCPeerConnection] = None
user_pc: Optional[RTCPeerConnection] = None

robot_video_track: Optional[MediaStreamTrack] = None
robot_audio_track: Optional[MediaStreamTrack] = None
robot_control_channel = None
robot_available_cameras: list[int] = []  # Camera IDs reported by robot
# Use unbuffered relay for lowest latency (no frame queuing)
relay = MediaRelay()

DEFAULT_ICE_SERVERS: list[str] = [
    "turn:47.242.85.149:3478?transport=udp",
    "turn:47.242.85.149:3478?transport=tcp",
    "stun:47.242.85.149:3478",
]


def _get_turn_credentials() -> tuple[str | None, str | None]:
    user = os.getenv("TURN_USER")
    password = os.getenv("TURN_PASS")
    if not user or not password:
        return None, None
    return user, password


def _get_ice_servers_urls() -> list[str]:
    env_val = os.getenv("ICE_SERVERS")
    servers: list[str] = list(DEFAULT_ICE_SERVERS)
    if env_val:
        try:
            parsed = json.loads(env_val)
            # Accept either list of strings or list of {"urls": [...]}.
            if isinstance(parsed, list) and parsed:
                if isinstance(parsed[0], str):
                    servers = [str(u) for u in parsed]
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
                        servers = collected
        except Exception:
            logger.warning("Invalid ICE_SERVERS env value; using default ICE list")
    return servers


def _get_ice_configuration() -> RTCConfiguration:
    servers = _get_ice_servers_urls()
    turn_user, turn_pass = _get_turn_credentials()

    if not servers:
        return RTCConfiguration(iceServers=[])

    if turn_user and turn_pass:
        ice_servers = [
            RTCIceServer(urls=servers, username=turn_user, credential=turn_pass)
        ]
    else:
        # Allow STUN-only usage if TURN creds are not configured.
        ice_servers = [RTCIceServer(urls=servers)]
    return RTCConfiguration(iceServers=ice_servers)


async def handle_get_ice(request: web.Request) -> web.Response:
    """Return ICE servers config for the browser (requires auth).

    Note: TURN credentials must be available to clients to use TURN.
    Keeping them out of the repo avoids accidental leaks.
    """
    if not _check_auth(request):
        return web.Response(status=401, text="Unauthorized")

    servers = _get_ice_servers_urls()
    turn_user, turn_pass = _get_turn_credentials()

    entry: dict = {"urls": servers}
    if turn_user and turn_pass:
        entry["username"] = turn_user
        entry["credential"] = turn_pass

    return web.Response(
        content_type="application/json",
        text=json.dumps({"iceServers": [entry]}),
    )


def _get_bind_config() -> tuple[str, int]:
    host = os.getenv("HOST", "0.0.0.0")
    port_str = os.getenv("CLOUD_SERVER_PORT") or os.getenv("PORT") or str(DEFAULT_PORT)
    try:
        port = int(port_str)
    except ValueError:
        logger.warning("Invalid port '%s', falling back to %s", port_str, DEFAULT_PORT)
        port = DEFAULT_PORT
    return host, port


def _build_ssl_context() -> Optional[ssl.SSLContext]:
    cert_file = os.getenv("SSL_CERT_FILE")
    key_file = os.getenv("SSL_KEY_FILE")
    password = os.getenv("SSL_PASSWORD")

    if not cert_file or not key_file:
        return None

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile=cert_file, keyfile=key_file, password=password)
    # Disallow legacy/weak options while keeping defaults sensible.
    ctx.options |= ssl.OP_NO_SSLv2 | ssl.OP_NO_SSLv3
    ctx.options |= ssl.OP_NO_COMPRESSION
    return ctx


STATIC_DIR = Path(__file__).parent / "static"
INDEX_FILE = STATIC_DIR / "index.html"
LOGIN_FILE = STATIC_DIR / "login.html"


def _check_auth(request: web.Request) -> bool:
    """Check if request has valid authentication."""
    if not AUTH_ENABLED:
        return True

    # Check session cookie
    session_token = request.cookies.get("session")
    if session_token and session_token in valid_sessions:
        return True

    return False


def _require_auth(handler):
    """Decorator to require authentication for a route."""

    async def wrapper(request: web.Request):
        if not _check_auth(request):
            # Redirect to login page for GET requests
            if request.method == "GET":
                raise web.HTTPFound("/login")
            # Return 401 for API requests
            return web.Response(status=401, text="Unauthorized")
        return await handler(request)

    return wrapper


async def handle_login_page(request: web.Request) -> web.Response:
    """Serve the login page."""
    if not AUTH_ENABLED:
        raise web.HTTPFound("/")

    # If already authenticated, redirect to main page
    if _check_auth(request):
        raise web.HTTPFound("/")

    return web.FileResponse(LOGIN_FILE)


async def handle_login(request: web.Request) -> web.Response:
    """Handle login POST request."""
    if not AUTH_ENABLED:
        return web.json_response({"success": True})

    try:
        data = await request.json()
        username = data.get("username", "")
        password = data.get("password", "")
    except Exception:
        return web.json_response(
            {"success": False, "error": "Invalid request"}, status=400
        )

    if username == AUTH_USERNAME and password == AUTH_PASSWORD:
        # Generate session token
        session_token = secrets.token_urlsafe(32)
        valid_sessions.add(session_token)

        response = web.json_response({"success": True})
        response.set_cookie(
            "session",
            session_token,
            max_age=86400 * 7,  # 7 days
            httponly=True,
            samesite="Lax",
        )
        logger.info("User '%s' logged in successfully", username)
        return response
    else:
        logger.warning("Failed login attempt for user '%s'", username)
        return web.json_response(
            {"success": False, "error": "Invalid credentials"}, status=401
        )


async def handle_logout(request: web.Request) -> web.Response:
    """Handle logout request."""
    session_token = request.cookies.get("session")
    if session_token and session_token in valid_sessions:
        valid_sessions.discard(session_token)

    response = web.HTTPFound("/login")
    response.del_cookie("session")
    return response


async def on_shutdown(app: web.Application):
    global robot_pc, user_pc
    if robot_pc:
        await robot_pc.close()
    if user_pc:
        await user_pc.close()


def _optimize_sdp_for_low_latency(sdp: str) -> str:
    """Modify SDP to optimize for low latency streaming."""
    lines = sdp.split("\r\n")
    optimized = []
    for line in lines:
        optimized.append(line)
        # Add low-latency parameters after video media line
        if line.startswith("m=video"):
            # Will add b=AS after media line in a=fmtp
            pass
    return "\r\n".join(optimized)


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
        global robot_control_channel, robot_available_cameras
        logger.info(f"Robot data channel received: {channel.label}")
        if channel.label == "control":
            robot_control_channel = channel

            @channel.on("message")
            def on_robot_message(message):
                global robot_available_cameras
                try:
                    data = json.loads(message)
                    if "available_cameras" in data:
                        robot_available_cameras = data["available_cameras"]
                        logger.info(
                            "Robot reported available cameras: %s",
                            robot_available_cameras,
                        )
                except Exception as e:
                    logger.debug("Error parsing robot message: %s", e)

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

    # Add Robot Tracks to User PC with unbuffered relay for lowest latency
    if robot_video_track:
        # buffered=False disables frame queuing for real-time streaming
        pc.addTrack(relay.subscribe(robot_video_track, buffered=False))
        logger.info("Added robot video track to user (unbuffered)")

    if robot_audio_track:
        pc.addTrack(relay.subscribe(robot_audio_track, buffered=False))
        logger.info("Added robot audio track to user (unbuffered)")

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
    """Serve the main page (requires auth)."""
    if not _check_auth(request):
        raise web.HTTPFound("/login")
    return web.FileResponse(INDEX_FILE)


async def protected_offer(request: web.Request) -> web.Response:
    """Protected user offer endpoint."""
    if not _check_auth(request):
        return web.Response(status=401, text="Unauthorized")
    return await handle_user_offer(request)


async def handle_get_cameras(request: web.Request) -> web.Response:
    """Return available cameras reported by the robot."""
    return web.Response(
        content_type="application/json",
        text=json.dumps({"cameras": robot_available_cameras}),
    )


def create_app() -> web.Application:
    app = web.Application()

    # Static assets (JS/CSS). Kept public; main page still requires auth.
    app.router.add_static("/static/", STATIC_DIR, show_index=False)

    # Public routes
    app.router.add_get("/login", handle_login_page)
    app.router.add_post("/api/login", handle_login)
    app.router.add_get("/logout", handle_logout)
    app.router.add_get("/api/cameras", handle_get_cameras)

    # Protected API
    app.router.add_get("/api/ice", handle_get_ice)

    # Protected routes
    app.router.add_get("/", index)
    app.router.add_post("/offer", protected_offer)

    # Robot API (no auth - robots use their own credentials)
    app.router.add_post("/robot/offer", handle_robot_offer)

    app.on_shutdown.append(on_shutdown)

    if AUTH_ENABLED:
        logger.info("Authentication enabled (user: %s)", AUTH_USERNAME)
    else:
        logger.info("Authentication disabled")

    return app


def run_server(host: Optional[str] = None, port: Optional[int] = None):
    logging.basicConfig(level=logging.INFO)
    bind_host, bind_port = _get_bind_config()
    if host:
        bind_host = host
    if port:
        bind_port = port

    ssl_context = _build_ssl_context()
    scheme = "https" if ssl_context else "http"
    logger.info("Starting Cloud Server on %s://%s:%s", scheme, bind_host, bind_port)
    if bind_port == 443 and not ssl_context:
        logger.warning(
            "Port 443 without TLS; set SSL_CERT_FILE and SSL_KEY_FILE to enable HTTPS"
        )

    web.run_app(create_app(), host=bind_host, port=bind_port, ssl_context=ssl_context)


if __name__ == "__main__":
    run_server()
