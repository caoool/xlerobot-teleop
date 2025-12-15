import os
import sys
import pathlib

# Add src directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# Load .env automatically when running directly (not via docker-compose env_file).
# This makes settings like ROBOT_AUDIO_INPUT_DEVICE take effect without manual export.
try:
    from dotenv import load_dotenv  # type: ignore

    _repo_root = pathlib.Path(__file__).resolve().parents[2]
    _env_path = _repo_root / ".env"
    if _env_path.exists():
        load_dotenv(dotenv_path=_env_path, override=False)
except Exception:
    # Optional dependency / best-effort only.
    pass

import asyncio
import json
import logging
import time
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
AUDIO_LATENCY_MS = int(
    os.getenv("ROBOT_AUDIO_LATENCY_MS", "10")
)  # Reduced for lower latency
AUDIO_PTIME_MS = int(os.getenv("ROBOT_AUDIO_PTIME_MS", "10"))  # Smaller packets
# Use Opus-native rate by default; 8k sounds extremely muffled.
AUDIO_RATE = os.getenv("ROBOT_AUDIO_RATE", "48000")
AUDIO_CHANNELS = os.getenv("ROBOT_AUDIO_CHANNELS", "1")

# Low-latency tuning constants
VIDEO_BITRATE_KBPS = int(os.getenv("ROBOT_VIDEO_BITRATE_KBPS", "1500"))

# Robust connection settings
RECONNECT_DELAY_MIN = 2  # Minimum delay between reconnection attempts
RECONNECT_DELAY_MAX = 30  # Maximum delay (with exponential backoff)
HEALTH_CHECK_INTERVAL = 5  # Seconds between health checks
CONNECTION_TIMEOUT = 15  # Timeout for HTTP requests
ICE_GATHERING_TIMEOUT = 10  # Timeout for ICE gathering
ERROR_LOG_INTERVAL = 60  # Only log same error once per interval

controller: Controller | None = None
_last_error_log: dict[str, float] = {}  # Track last error log time by message type


def _log_error_throttled(error_type: str, message: str, *args):
    """Log error with rate limiting to avoid spam."""
    now = time.time()
    last_log = _last_error_log.get(error_type, 0)
    if now - last_log >= ERROR_LOG_INTERVAL:
        logger.error(message, *args)
        _last_error_log[error_type] = now
    else:
        logger.debug(message, *args)


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
        "thread_queue_size": "512",  # Smaller queue for lower latency
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


class RobustRobotClient:
    """Robust robot client that maintains persistent connection to server.

    Features:
    - Automatic reconnection with exponential backoff
    - Proper resource cleanup between connections
    - Health monitoring and ICE restart
    - Camera persistence across reconnections
    """

    def __init__(self, server_url: str, camera_ids: list[int]):
        self.server_url = server_url
        self.camera_ids = camera_ids
        self.pc: Optional[RTCPeerConnection] = None
        self.video_track: Optional[MultiCameraVideoTrack] = None
        self.audio_player: Optional[MediaPlayer] = None
        self.recorder: Optional[MediaRecorder] = None
        self.control_channel = None
        self.ice_config: Optional[RTCConfiguration] = None
        self._running = False
        self._connection_healthy = False
        self._reconnect_delay = RECONNECT_DELAY_MIN
        self._last_ice_restart = 0
        self._connection_start_time = 0
        self._stats_task: Optional[asyncio.Task] = None

        # Detect available cameras early, before any are opened
        self._available_cameras = self._detect_available_cameras()

        # Initialize controller
        global controller
        if controller is None:
            controller = Controller()
        self.controller = controller

        # Setup ICE configuration
        self._setup_ice_config()

    def _setup_ice_config(self):
        """Setup ICE servers configuration."""
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
                logger.warning("Invalid ROBOT_ICE_SERVERS, using defaults")

        turn_user = os.getenv("TURN_USER")
        turn_pass = os.getenv("TURN_PASS")

        if not ice_urls:
            ice_servers = []
        elif turn_user and turn_pass:
            ice_servers = [
                RTCIceServer(urls=ice_urls, username=turn_user, credential=turn_pass)
            ]
        else:
            ice_servers = [RTCIceServer(urls=ice_urls)]
        self.ice_config = RTCConfiguration(iceServers=ice_servers)

    def _detect_available_cameras(self, max_cameras: int = 10) -> list[int]:
        """Detect available camera IDs by probing video devices."""
        import cv2

        found = []
        for cam_id in range(max_cameras):
            try:
                cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
                if cap.isOpened():
                    ret, _ = cap.read()
                    cap.release()
                    if ret:
                        found.append(cam_id)
            except Exception:
                pass
        logger.info("Detected available cameras: %s", found)
        return found

    async def _cleanup_resources(self):
        """Clean up all WebRTC and media resources."""
        logger.info("Cleaning up resources...")

        # Cancel stats task
        if self._stats_task and not self._stats_task.done():
            self._stats_task.cancel()
            try:
                await self._stats_task
            except asyncio.CancelledError:
                pass
        self._stats_task = None

        # Close peer connection first
        if self.pc:
            try:
                await self.pc.close()
            except Exception as e:
                logger.debug("Error closing peer connection: %s", e)
            self.pc = None

        # Stop recorder
        if self.recorder:
            try:
                await self.recorder.stop()
            except Exception as e:
                logger.debug("Error stopping recorder: %s", e)
            self.recorder = None

        # Stop video track (releases camera)
        if self.video_track:
            try:
                self.video_track.stop()
            except Exception as e:
                logger.debug("Error stopping video track: %s", e)
            self.video_track = None

        # Stop audio player
        if self.audio_player:
            try:
                if hasattr(self.audio_player, "stop"):
                    self.audio_player.stop()
            except Exception as e:
                logger.debug("Error stopping audio player: %s", e)
            self.audio_player = None

        self.control_channel = None
        self._connection_healthy = False

        # Brief pause to ensure resources are released
        await asyncio.sleep(0.5)
        logger.info("Resources cleaned up")

    def _create_video_track(self) -> Optional[MultiCameraVideoTrack]:
        """Create a fresh video track with camera."""
        try:
            track = MultiCameraVideoTrack(camera_ids=self.camera_ids)
            logger.info("Video track created successfully")
            return track
        except Exception as e:
            logger.error("Failed to create video track: %s", e)
            return None

    def _setup_control_channel_handlers(self, channel):
        """Setup handlers for the control data channel."""

        @channel.on("open")
        def on_open():
            logger.info("Control channel open")
            self._connection_healthy = True
            # Send available cameras to server (detected at init, before camera opened)
            if self._available_cameras:
                try:
                    channel.send(
                        json.dumps({"available_cameras": self._available_cameras})
                    )
                    logger.info(
                        "Sent available cameras to server: %s", self._available_cameras
                    )
                except Exception as e:
                    logger.error("Failed to send camera list: %s", e)

        @channel.on("close")
        def on_close():
            logger.info("Control channel closed")
            self._connection_healthy = False

        @channel.on("message")
        def on_message(message):
            logger.debug("Received control message: %s", message)
            try:
                data = json.loads(message)
                if data.get("camera_config") is not None:
                    cfg = data.get("camera_config") or {}
                    try:
                        cam_index = int(cfg.get("index", 0))
                        width = int(
                            cfg.get(
                                "width",
                                self.video_track.width if self.video_track else 640,
                            )
                        )
                        height = int(
                            cfg.get(
                                "height",
                                self.video_track.height if self.video_track else 480,
                            )
                        )
                        fps = float(
                            cfg.get(
                                "fps", self.video_track.fps if self.video_track else 20
                            )
                        )
                        if self.video_track:
                            self.video_track.set_camera_config(
                                cam_index, width, height, fps
                            )
                    except Exception as exc:
                        logger.error("Failed to set camera config: %s", exc)
                elif "joycon_arms" in data:
                    # Handle Joy-Con arm control messages
                    arms_data = data["joycon_arms"]
                    left_arm = arms_data.get("left")
                    right_arm = arms_data.get("right")
                    self.controller.move_arms(
                        left_arm=left_arm,
                        right_arm=right_arm,
                    )
                elif "pan" in data and "tilt" in data:
                    self.controller.move_head(
                        pan_deg=float(data["pan"]), tilt_deg=float(data["tilt"])
                    )
                elif "x" in data:  # base move
                    self.controller.move_base(
                        x_vel=float(data.get("x", 0)),
                        theta_vel=float(data.get("theta", 0)),
                        y_vel=float(data.get("y", 0)),
                    )
            except Exception as e:
                logger.error("Error processing control message: %s", e)

    async def _monitor_connection(self):
        """Monitor connection health and handle issues."""
        while self._running and self.pc:
            await asyncio.sleep(HEALTH_CHECK_INTERVAL)

            if not self.pc:
                break

            state = self.pc.connectionState
            ice_state = self.pc.iceConnectionState

            logger.debug("Connection state: %s, ICE state: %s", state, ice_state)

            # Check if connection is healthy
            if state in ("connected", "completed"):
                self._connection_healthy = True
                self._reconnect_delay = RECONNECT_DELAY_MIN  # Reset backoff on success
            elif state == "disconnected":
                # Try ICE restart if we haven't recently
                now = time.time()
                if now - self._last_ice_restart > 30:
                    logger.info("Connection disconnected, attempting ICE restart...")
                    self._last_ice_restart = now
                    # ICE restart is not directly supported in aiortc for established connections
                    # We'll let the connection attempt recovery naturally
                    await asyncio.sleep(5)
                    if self.pc and self.pc.connectionState == "disconnected":
                        logger.warning("ICE restart failed, will reconnect")
                        self._connection_healthy = False
                        break
            elif state in ("failed", "closed"):
                logger.warning("Connection %s, will reconnect", state)
                self._connection_healthy = False
                break

    async def _connect(self) -> bool:
        """Establish connection to the server. Returns True on success."""
        await self._cleanup_resources()

        # Create fresh peer connection
        self.pc = RTCPeerConnection(configuration=self.ice_config)

        # Track connection state changes
        connection_established = asyncio.Event()
        connection_failed = asyncio.Event()

        @self.pc.on("connectionstatechange")
        async def on_connectionstatechange():
            state = self.pc.connectionState
            logger.info("Connection state: %s", state)
            if state == "connected":
                connection_established.set()
            elif state in ("failed", "closed"):
                connection_failed.set()

        @self.pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            logger.info("ICE connection state: %s", self.pc.iceConnectionState)

        # Add video track
        self.video_track = self._create_video_track()
        if self.video_track:
            self.pc.addTrack(self.video_track)
            logger.info("Added video track")
        else:
            logger.error("No video track available")

        # Audio: always create a transceiver that can RECEIVE (speaker) and optionally SEND (mic).
        # Without an audio transceiver offering recv, the robot cannot receive user audio.
        audio_transceiver = self.pc.addTransceiver("audio", direction="sendrecv")
        self.audio_player = _open_media_player("input")
        if self.audio_player and self.audio_player.audio:
            await audio_transceiver.sender.replaceTrack(self.audio_player.audio)
            logger.info("Audio transceiver ready (sendrecv) with mic")
        else:
            logger.info("Audio transceiver ready (sendrecv) without mic")

        # Create control data channel
        control_channel = self.pc.createDataChannel(
            "control",
            ordered=False,
            maxRetransmits=0,
        )
        self.control_channel = control_channel
        self._setup_control_channel_handlers(control_channel)

        # Handle incoming audio track
        @self.pc.on("track")
        def on_track(track):
            logger.info("Received track: %s", track.kind)
            if track.kind == "audio":
                self.recorder = _open_media_recorder("output", track)
                if self.recorder:
                    asyncio.ensure_future(self.recorder.start())
                    logger.info("Started recording audio to speakers")

        # Create and send offer
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)

        logger.info("Connecting to %s...", self.server_url)

        try:
            timeout = aiohttp.ClientTimeout(total=CONNECTION_TIMEOUT)
            async with aiohttp.ClientSession(timeout=timeout) as session:
                async with session.post(
                    f"{self.server_url}/robot/offer",
                    json={
                        "sdp": self.pc.localDescription.sdp,
                        "type": self.pc.localDescription.type,
                    },
                ) as resp:
                    if resp.status != 200:
                        error_text = await resp.text()
                        logger.error("Server rejected offer: %s", error_text)
                        return False
                    answer = await resp.json()

            await self.pc.setRemoteDescription(
                RTCSessionDescription(sdp=answer["sdp"], type=answer["type"])
            )
            logger.info("Remote description set, waiting for connection...")

            # Wait for connection with timeout
            try:
                done, pending = await asyncio.wait(
                    [
                        asyncio.create_task(connection_established.wait()),
                        asyncio.create_task(connection_failed.wait()),
                    ],
                    timeout=ICE_GATHERING_TIMEOUT,
                    return_when=asyncio.FIRST_COMPLETED,
                )

                for task in pending:
                    task.cancel()

                if connection_failed.is_set():
                    logger.error("Connection failed during ICE negotiation")
                    return False

                if not connection_established.is_set():
                    logger.warning("Connection timeout, but may still succeed")
                    # Don't return False here, connection might still complete

            except asyncio.TimeoutError:
                logger.warning("ICE gathering timeout")

            self._connection_start_time = time.time()
            self._connection_healthy = True
            logger.info("Connected to server successfully")
            return True

        except aiohttp.ClientError as e:
            _log_error_throttled(
                "http_connection",
                "HTTP connection error: %s (server may be down or unreachable)",
                e,
            )
            return False
        except Exception as e:
            _log_error_throttled("connection", "Connection error: %s", e)
            return False

    async def run(self):
        """Main run loop - maintains persistent connection."""
        self._running = True
        logger.info("Starting robust robot client...")

        while self._running:
            # Attempt connection
            success = await self._connect()

            if success:
                # Reset backoff on successful connection
                self._reconnect_delay = RECONNECT_DELAY_MIN

                # Start monitoring task
                self._stats_task = asyncio.create_task(self._monitor_connection())

                # Keep alive while connected
                try:
                    while self._running and self.pc:
                        await asyncio.sleep(1)

                        pc = self.pc
                        if not pc:
                            break

                        # Check if connection is still healthy
                        if pc.connectionState in ("failed", "closed"):
                            logger.warning("Connection lost: %s", pc.connectionState)
                            break

                        # Check if we've been disconnected for too long
                        if pc.connectionState == "disconnected":
                            await asyncio.sleep(5)  # Give it a chance to recover
                            if self.pc and self.pc.connectionState == "disconnected":
                                logger.warning("Connection stalled, reconnecting...")
                                break

                except asyncio.CancelledError:
                    logger.info("Client run cancelled")
                    break
            else:
                # Exponential backoff on failure
                self._reconnect_delay = min(
                    self._reconnect_delay * 1.5, RECONNECT_DELAY_MAX
                )

            # Cleanup before retry
            await self._cleanup_resources()

            if self._running:
                # Log reconnection at debug level to avoid spam during server downtime
                if self._reconnect_delay > RECONNECT_DELAY_MIN:
                    logger.debug(
                        "Reconnecting in %.1f seconds...", self._reconnect_delay
                    )
                else:
                    logger.info(
                        "Reconnecting in %.1f seconds...", self._reconnect_delay
                    )
                await asyncio.sleep(self._reconnect_delay)

        # Final cleanup
        await self._cleanup_resources()
        logger.info("Robot client stopped")

    async def stop(self):
        """Stop the client gracefully."""
        logger.info("Stopping robot client...")
        self._running = False
        await self._cleanup_resources()


async def run_robot_client(server_url: str, camera_ids: list[int]):
    """Run the robust robot client."""
    client = RobustRobotClient(server_url, camera_ids)

    # Handle graceful shutdown
    loop = asyncio.get_event_loop()

    def signal_handler():
        logger.info("Shutdown signal received")
        asyncio.create_task(client.stop())

    try:
        import signal

        for sig in (signal.SIGTERM, signal.SIGINT):
            loop.add_signal_handler(sig, signal_handler)
    except (NotImplementedError, ValueError):
        # Windows doesn't support add_signal_handler
        pass

    await client.run()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    server_url = os.getenv("CLOUD_SERVER_URL", "http://localhost:8080")
    # Default camera ID; override with ROBOT_CAMERA_IDS="0" (first value used)
    env_ids = os.getenv("ROBOT_CAMERA_IDS")
    if env_ids:
        try:
            camera_ids = [int(x) for x in env_ids.split(",") if x.strip()]
        except ValueError:
            logger.warning(
                "Invalid ROBOT_CAMERA_IDS value '%s', falling back to defaults", env_ids
            )
            camera_ids = [0]
    else:
        camera_ids = [0]
    asyncio.run(run_robot_client(server_url, camera_ids))
