import os
import asyncio
import json
import logging
import pathlib
from typing import Iterable, List, Optional, Tuple

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer, MediaRecorder

from camera import MultiCameraVideoTrack
from robot.controller import Controller

logger = logging.getLogger(__name__)

# Track active peer connections for cleanup
pcs: set[RTCPeerConnection] = set()
audio_recorders: dict[RTCPeerConnection, MediaRecorder] = {}
audio_players: dict[RTCPeerConnection, MediaPlayer] = {}
_logged_audio_in_unavailable = False
_logged_audio_out_unavailable = False
controller: Controller | None = None
DISABLE_AUDIO = os.getenv("ROBOT_DISABLE_AUDIO") == "1"
AUDIO_LATENCY_MS = int(os.getenv("ROBOT_AUDIO_LATENCY_MS", "30"))
AUDIO_PTIME_MS = int(os.getenv("ROBOT_AUDIO_PTIME_MS", "20"))
AUDIO_RATE = os.getenv("ROBOT_AUDIO_RATE", "16000")
AUDIO_CHANNELS = os.getenv("ROBOT_AUDIO_CHANNELS", "1")


def _ensure_alsa_config_env():
    """Set ALSA_CONFIG_PATH to a common default if missing and present on disk."""
    if os.getenv("ALSA_CONFIG_PATH"):
        return
    default_path = pathlib.Path("/usr/share/alsa/alsa.conf")
    if default_path.exists():
        os.environ["ALSA_CONFIG_PATH"] = str(default_path)
        logger.debug("ALSA_CONFIG_PATH set to %s", default_path)


def _get_env_audio_spec(kind: str) -> Tuple[Optional[str], Optional[str]]:
    """Return (device, format) from env for given kind ('input'|'output')."""
    dev = os.getenv(f"ROBOT_AUDIO_{kind.upper()}_DEVICE")
    fmt = os.getenv(f"ROBOT_AUDIO_{kind.upper()}_FORMAT")
    return dev, fmt


def _open_media_player(kind: str) -> Optional[MediaPlayer]:
    """Try to open a MediaPlayer for audio input.

    Order: env override -> pulse default -> alsa default. Logs only once on failure.
    """
    if DISABLE_AUDIO:
        logger.info("Audio input disabled via ROBOT_DISABLE_AUDIO=1")
        return None
    _ensure_alsa_config_env()
    global _logged_audio_in_unavailable
    env_dev, env_fmt = _get_env_audio_spec("input")
    attempts = []
    if env_dev and env_fmt:
        attempts.append((env_dev, env_fmt, "env override"))
    attempts += [("default", "pulse", "PulseAudio default"), ("default", "alsa", "ALSA default")]

    options = {
        "audio_buffer_size": str(AUDIO_LATENCY_MS),
        "sample_rate": AUDIO_RATE,
        "channels": AUDIO_CHANNELS
    }

    for device, fmt, label in attempts:
        try:
            player = MediaPlayer(device, format=fmt, options=options)
            if player.audio:
                logger.info("Audio input ready via %s (%s,%s)", label, device, fmt)
                return player
        except Exception as exc:
            logger.debug("Audio input %s unavailable (%s,%s): %s", label, device, fmt, exc)

    if not _logged_audio_in_unavailable:
        logger.warning("Audio input unavailable (tried env override, PulseAudio, ALSA); set ROBOT_AUDIO_INPUT_DEVICE/FORMAT if you have a specific device.")
        _logged_audio_in_unavailable = True
    return None


def _open_media_recorder(kind: str, track) -> Optional[MediaRecorder]:
    """Try to open MediaRecorder for audio output (play to speakers).

    Order: env override -> pulse default -> alsa default.
    """
    if DISABLE_AUDIO:
        logger.info("Audio output disabled via ROBOT_DISABLE_AUDIO=1")
        return None
    _ensure_alsa_config_env()
    global _logged_audio_out_unavailable
    env_dev, env_fmt = _get_env_audio_spec("output")
    attempts = []
    if env_dev and env_fmt:
        attempts.append((env_dev, env_fmt, "env override"))
    attempts += [("default", "pulse", "PulseAudio default"), ("default", "alsa", "ALSA default")]

    options = {
        "audio_buffer_size": str(AUDIO_LATENCY_MS),
        "sample_rate": AUDIO_RATE,
        "channels": AUDIO_CHANNELS
    }

    for device, fmt, label in attempts:
        try:
            recorder = MediaRecorder(device, format=fmt, options=options)
            recorder.addTrack(track)
            logger.info("Audio output ready via %s (%s,%s)", label, device, fmt)
            return recorder
        except Exception as exc:
            logger.debug("Audio output %s unavailable (%s,%s): %s", label, device, fmt, exc)

    if not _logged_audio_out_unavailable:
        logger.warning("Audio output unavailable (tried env override, PulseAudio, ALSA); set ROBOT_AUDIO_OUTPUT_DEVICE/FORMAT if you have a specific device.")
        _logged_audio_out_unavailable = True
    return None


async def handle_offer(request: web.Request, camera_ids: Iterable[int]) -> web.Response:
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    pcs.add(pc)

    # Optional audio capture from robot microphone
    audio_player: MediaPlayer | None = _open_media_player("input")

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        logger.info("Connection state is %s", pc.connectionState)
        if pc.connectionState in {"failed", "closed"}:
            rec = audio_recorders.pop(pc, None)
            if rec:
                try:
                    await rec.stop()
                except Exception:
                    pass
            player = audio_players.pop(pc, None)
            if player:
                try:
                    player.stop()
                except Exception:
                    pass
            await pc.close()
            pcs.discard(pc)

    try:
        video_track = MultiCameraVideoTrack(camera_ids=list(camera_ids))
        sender = pc.addTrack(video_track)

        # Apply optional caps if the aiortc version supports sender parameter APIs
        max_bitrate_env = os.getenv("ROBOT_VIDEO_MAX_BITRATE")
        max_fps_env = os.getenv("ROBOT_CAMERA_FPS")
        if (max_bitrate_env or max_fps_env) and hasattr(sender, "getParameters") and hasattr(sender, "setParameters"):
            params = sender.getParameters()
            max_bitrate = int(max_bitrate_env) if max_bitrate_env else None
            max_fps = float(max_fps_env) if max_fps_env else None
            for enc in params.encodings or []:
                if max_bitrate:
                    enc.maxBitrate = max_bitrate
                if max_fps:
                    enc.maxFramerate = max_fps
            if params.encodings:
                await sender.setParameters(params)
        logger.info("Added multi-camera video track to peer connection")
    except Exception as exc:
        logger.error("Error adding video track: %s", exc)
        return web.Response(
            content_type="application/json",
            text=json.dumps({"error": str(exc)}),
            status=500,
        )

    # Add audio track from robot mic to client if available
    if audio_player and audio_player.audio:
        try:
            sender = pc.addTrack(audio_player.audio)
            if hasattr(sender, "getParameters") and hasattr(sender, "setParameters"):
                params = sender.getParameters()
                for enc in params.encodings or []:
                    enc.ptime = AUDIO_PTIME_MS
                    enc.maxBitrate = 32000
                if params.encodings:
                    await sender.setParameters(params)
            audio_players[pc] = audio_player
            logger.info("Added audio track from robot microphone")
        except Exception as exc:
            logger.warning("Failed to add audio track: %s", exc)
            audio_player = None

    # Play client audio to robot speakers
    @pc.on("track")
    async def on_track(track):
        if track.kind == "audio":
            recorder = _open_media_recorder("output", track)
            if recorder:
                try:
                    await recorder.start()
                    audio_recorders[pc] = recorder
                    logger.info("Audio track received; streaming to robot speakers")
                except Exception as exc:
                    logger.debug("Failed to start audio recorder: %s", exc)

        @track.on("ended")
        async def on_ended():  # pragma: no cover
            rec = audio_recorders.pop(pc, None)
            if rec:
                await rec.stop()
            player = audio_players.pop(pc, None)
            if player:
                player.stop()

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type,
        }),
    )


def render_index(camera_ids: List[int]) -> str:
    ids_str = ", ".join(map(str, camera_ids))
    bottom_ids = camera_ids[1:3]
    bottom_label = f"Camera {bottom_ids[0]} and Camera {bottom_ids[1]}" if len(bottom_ids) == 2 else "Aux cameras"
    return f"""
<!DOCTYPE html>
<html>
<head>
    <title>WebRTC Multi-Camera Stream</title>
    <style>
        body {{
            font-family: Arial, sans-serif;
            max-width: 1320px;
            margin: 20px auto;
            padding: 20px;
            background-color: #f0f0f0;
        }}
        h1 {{ color: #333; }}
        #video {{
            width: 100%;
            max-width: 1280px;
            background-color: #000;
            border-radius: 8px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }}
        button {{
            margin: 10px 5px;
            padding: 10px 20px;
            font-size: 16px;
            cursor: pointer;
            background-color: #4CAF50;
            color: white;
            border: none;
            border-radius: 4px;
        }}
        button:hover {{ background-color: #45a049; }}
        button:disabled {{
            background-color: #cccccc;
            cursor: not-allowed;
        }}
        #status {{
            margin-top: 10px;
            padding: 10px;
            border-radius: 4px;
            background-color: #fff;
        }}
        .info {{
            margin: 10px 0;
            padding: 10px;
            background-color: #e3f2fd;
            border-radius: 4px;
            font-size: 14px;
        }}
        .stats {{
            margin-top: 10px;
            padding: 10px;
            background-color: #fff;
            border-radius: 4px;
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
            gap: 8px;
        }}
        .stat {{
            font-size: 14px;
            color: #333;
        }}
    </style>
</head>
<body>
    <h1>WebRTC Multi-Camera Stream</h1>
    <div class="info">
        <strong>Camera Layout:</strong> Main camera (ID {camera_ids[0]}) on top, {bottom_label} split at bottom.<br>
        <strong>Configured camera IDs:</strong> {ids_str}<br>
        <strong>Controls:</strong> W/S tilt up/down, A/D pan left/right (head); Arrow keys drive base (forward/back + rotate)
    </div>
    <div>
        <button id="startButton" onclick="start()">Start Stream</button>
        <button id="stopButton" onclick="stop()" disabled>Stop Stream</button>
    </div>
    <div id="status">Status: Not connected</div>
    <div class="stats" id="stats">
        <div class="stat" id="stat-resolution">Resolution: -</div>
        <div class="stat" id="stat-fps">FPS: -</div>
        <div class="stat" id="stat-latency">RTT (ms): -</div>
    </div>
    <br>
    <video id="video" autoplay playsinline></video>

    <script>
        let pc = null;
        let localStream = null;
        const video = document.getElementById('video');
        const startButton = document.getElementById('startButton');
        const stopButton = document.getElementById('stopButton');
        const status = document.getElementById('status');
        const statResolution = document.getElementById('stat-resolution');
        const statFps = document.getElementById('stat-fps');
        const statLatency = document.getElementById('stat-latency');

        let headPan = 0;
        let headTilt = 0;
        const PAN_STEP = 3;
        const TILT_STEP = 3;
        const PAN_LIMIT = 45;
        const TILT_LIMIT = 45;

        let statsInterval = null;
        let lastFramesDecoded = null;
        let lastTimestamp = null;

        // Base control state
        let baseX = 0;
        let baseY = 0;
        let baseTheta = 0;
        const BASE_X_STEP = 0.35; // m/s (bigger step to feel responsive)
        const BASE_THETA_STEP = 120; // deg/s (faster spins)

        function updateStatus(message) {{
            status.textContent = 'Status: ' + message;
            console.log(message);
        }}

        async function start() {{
            startButton.disabled = true;
            updateStatus('Connecting...');

            pc = new RTCPeerConnection({{
                iceServers: [{{urls: 'stun:stun.l.google.com:19302'}}]
            }});

            pc.addTransceiver('video', {{ direction: 'recvonly' }});
            pc.addTransceiver('audio', {{ direction: 'sendrecv' }});

            // Capture browser microphone to send to robot
            try {{
                localStream = await navigator.mediaDevices.getUserMedia({{ audio: true, video: false }});
                for (const track of localStream.getAudioTracks()) {{
                    pc.addTrack(track, localStream);
                }}
            }} catch (err) {{
                console.warn('Mic capture failed:', err);
            }}

            const remoteAudio = new Audio();
            remoteAudio.autoplay = true;

            pc.addEventListener('track', (event) => {{
                if (event.track.kind === 'video') {{
                    updateStatus('Receiving video stream');
                    video.srcObject = event.streams[0];
                }} else if (event.track.kind === 'audio') {{
                    // Play robot audio via an audio element
                    remoteAudio.srcObject = event.streams[0];
                }}
            }});

            pc.addEventListener('connectionstatechange', () => {{
                updateStatus('Connection state: ' + pc.connectionState);
                if (pc.connectionState === 'connected') {{
                    stopButton.disabled = false;
                }}
            }});

            if (statsInterval) {{
                clearInterval(statsInterval);
            }}
            statsInterval = setInterval(updateStats, 1000);

            const offer = await pc.createOffer();
            await pc.setLocalDescription(offer);

            try {{
                const response = await fetch('/offer', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify({{
                        sdp: pc.localDescription.sdp,
                        type: pc.localDescription.type,
                    }}),
                }});

                const answer = await response.json();
                if (answer.error) {{
                    updateStatus('Error: ' + answer.error);
                    startButton.disabled = false;
                    return;
                }}

                await pc.setRemoteDescription(answer);
                updateStatus('Connection established');
            }} catch (error) {{
                updateStatus('Error: ' + error.message);
                startButton.disabled = false;
            }}
        }}

        async function stop() {{
            stopButton.disabled = true;
            if (pc) {{
                pc.close();
                pc = null;
            }}
            if (localStream) {{
                localStream.getTracks().forEach(t => t.stop());
                localStream = null;
            }}
            video.srcObject = null;
            updateStatus('Disconnected');
            startButton.disabled = false;
            if (statsInterval) {{
                clearInterval(statsInterval);
                statsInterval = null;
            }}
            statResolution.textContent = 'Resolution: -';
            statFps.textContent = 'FPS: -';
            statLatency.textContent = 'RTT (ms): -';
        }}

        function clamp(value, min, max) {{
            return Math.min(Math.max(value, min), max);
        }}

        async function sendHeadMove(pan, tilt) {{
            try {{
                await fetch('/head_move', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify({{ pan, tilt }}),
                }});
            }} catch (err) {{
                console.warn('head_move error', err);
            }}
        }}

        async function sendBaseMove(x, y, theta) {{
            try {{
                await fetch('/base_move', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify({{ x, y, theta }}),
                }});
            }} catch (err) {{
                console.warn('base_move error', err);
            }}
        }}

        window.addEventListener('keydown', (e) => {{
            const key = e.key.toLowerCase();
            let changed = false;
            if (key === 'a') {{
                headPan = clamp(headPan - PAN_STEP, -PAN_LIMIT, PAN_LIMIT);
                changed = true;
            }} else if (key === 'd') {{
                headPan = clamp(headPan + PAN_STEP, -PAN_LIMIT, PAN_LIMIT);
                changed = true;
            }} else if (key === 'w') {{
                headTilt = clamp(headTilt + TILT_STEP, -TILT_LIMIT, TILT_LIMIT);
                changed = true;
            }} else if (key === 's') {{
                headTilt = clamp(headTilt - TILT_STEP, -TILT_LIMIT, TILT_LIMIT);
                changed = true;
            }}

            // Base drive (arrow keys)
            if (key === 'arrowup') {{
                baseX = BASE_X_STEP;
                changed = true;
            }} else if (key === 'arrowdown') {{
                baseX = -BASE_X_STEP;
                changed = true;
            }} else if (key === 'arrowleft') {{
                baseTheta = BASE_THETA_STEP;
                changed = true;
            }} else if (key === 'arrowright') {{
                baseTheta = -BASE_THETA_STEP;
                changed = true;
            }}

            if (changed) {{
                // Only send the relevant command to reduce chatter.
                if (key === 'a' || key === 'd' || key === 'w' || key === 's') {{
                    sendHeadMove(headPan, headTilt);
                }} else {{
                    sendBaseMove(baseX, baseY, baseTheta);
                }}
            }}
        }});

        window.addEventListener('keyup', (e) => {{
            const key = e.key.toLowerCase();
            let changed = false;

            // Stop base motion when arrow keys are released
            if (['arrowup', 'arrowdown'].includes(key)) {{
                baseX = 0;
                changed = true;
            }}
            if (['arrowleft', 'arrowright'].includes(key)) {{
                baseTheta = 0;
                changed = true;
            }}

            if (changed) {{
                sendBaseMove(baseX, baseY, baseTheta);
            }}
        }});

        async function updateStats() {{
            if (!pc) return;
            try {{
                const stats = await pc.getStats();
                let framesDecoded = null;
                let ts = null;
                let rttMs = null;

                stats.forEach(report => {{
                    if (report.type === 'inbound-rtp' && report.kind === 'video') {{
                        framesDecoded = report.framesDecoded ?? framesDecoded;
                        ts = report.timestamp ?? ts;
                    }}
                    if (report.type === 'candidate-pair' && report.state === 'succeeded' && report.currentRoundTripTime !== undefined) {{
                        rttMs = Math.round(report.currentRoundTripTime * 1000);
                    }}
                }});

                // Resolution from the video element once metadata is available
                if (video.videoWidth && video.videoHeight) {{
                    statResolution.textContent = `Resolution: ${{video.videoWidth}}x${{video.videoHeight}}`;
                }}

                // FPS from framesDecoded delta over time
                if (framesDecoded !== null && ts !== null) {{
                    if (lastFramesDecoded !== null && lastTimestamp !== null) {{
                        const deltaFrames = framesDecoded - lastFramesDecoded;
                        const deltaMs = ts - lastTimestamp;
                        if (deltaMs > 0) {{
                            const fps = (deltaFrames * 1000) / deltaMs;
                            statFps.textContent = `FPS: ${{fps.toFixed(1)}}`;
                        }}
                    }}
                    lastFramesDecoded = framesDecoded;
                    lastTimestamp = ts;
                }}

                if (rttMs !== null) {{
                    statLatency.textContent = `RTT (ms): ${{rttMs}}`;
                }}
            }} catch (err) {{
                console.warn('Stats error', err);
            }}
        }}
    </script>
</body>
</html>
    """


async def index(request: web.Request, camera_ids: Iterable[int]) -> web.Response:
    return web.Response(content_type="text/html", text=render_index(list(camera_ids)))


async def on_shutdown(app: web.Application):
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()
    # Stop any active audio recorders
    for rec in list(audio_recorders.values()):
        try:
            await rec.stop()
        except Exception:
            pass
    audio_recorders.clear()
    for player in list(audio_players.values()):
        try:
            player.stop()
        except Exception:
            pass
    audio_players.clear()


async def head_move(request: web.Request) -> web.Response:
    data = await request.json()
    pan = float(data.get("pan", 0.0))
    tilt = float(data.get("tilt", 0.0))
    if controller is None:
        return web.Response(status=500, text="Controller not initialized")
    try:
        action = controller.move_head(pan_deg=pan, tilt_deg=tilt)
        return web.Response(
            content_type="application/json",
            text=json.dumps({"ok": True, "action": action}),
        )
    except Exception as exc:  # pragma: no cover
        logger.error("head_move failed: %s", exc)
        return web.Response(
            status=500,
            content_type="application/json",
            text=json.dumps({"ok": False, "error": str(exc)}),
        )


async def base_move(request: web.Request) -> web.Response:
    data = await request.json()
    x_vel = float(data.get("x", 0.0))
    y_vel = float(data.get("y", 0.0))
    theta_vel = float(data.get("theta", 0.0))
    if controller is None:
        return web.Response(status=500, text="Controller not initialized")
    try:
        action = controller.move_base(x_vel=x_vel, y_vel=y_vel, theta_vel=theta_vel)
        return web.Response(
            content_type="application/json",
            text=json.dumps({"ok": True, "action": action}),
        )
    except Exception as exc:  # pragma: no cover
        logger.error("base_move failed: %s", exc)
        return web.Response(
            status=500,
            content_type="application/json",
            text=json.dumps({"ok": False, "error": str(exc)}),
        )


def create_app(camera_ids: Iterable[int] = (0, 2, 4)) -> web.Application:
    camera_ids = list(camera_ids)
    global controller
    if controller is None:
        controller = Controller()
    app = web.Application()
    app.router.add_get("/", lambda request: index(request, camera_ids))
    app.router.add_post("/offer", lambda request: handle_offer(request, camera_ids))
    app.router.add_post("/head_move", head_move)
    app.router.add_post("/base_move", base_move)
    app.on_shutdown.append(on_shutdown)
    return app


def run_server(host: str = "0.0.0.0", port: int = 8080, camera_ids: Iterable[int] = (0, 2, 4)):
    logger.info("Starting WebRTC multi-camera streaming server...")
    logger.info("Open http://%s:%s in your browser", host, port)
    web.run_app(create_app(camera_ids), host=host, port=port)
