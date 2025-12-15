import cv2
import logging
import numpy as np
import os
import time
from fractions import Fraction
from aiortc import VideoStreamTrack
from av import VideoFrame

logger = logging.getLogger(__name__)

# Low-latency tuning constants
CAMERA_BUFFER_SIZE = 1  # Minimum buffer to get latest frame
DEFAULT_CODEC = "MJPG"  # Hardware-accelerated, low latency
FRAME_INTERVAL_TOLERANCE = 0.5  # Allow some timing flexibility

# Robust camera settings
CAMERA_OPEN_RETRIES = 3  # Number of times to retry opening camera
CAMERA_OPEN_DELAY = 0.5  # Delay between open retries
CAMERA_REOPEN_THRESHOLD = 10  # Consecutive failures before reopening camera

# Error log rate-limiting (seconds)
CAMERA_ERROR_LOG_INTERVAL = 5.0
_last_camera_error_log: dict[str, float] = {}


def _log_camera_error_throttled(error_key: str, message: str, *args):
    """Log camera error with rate limiting to avoid spam."""
    now = time.time()
    last_log = _last_camera_error_log.get(error_key, 0)
    if now - last_log >= CAMERA_ERROR_LOG_INTERVAL:
        logger.error(message, *args)
        _last_camera_error_log[error_key] = now
    else:
        logger.debug(message, *args)


class CameraVideoTrack(VideoStreamTrack):
    """Single-camera video track using OpenCV with low-latency settings."""

    def __init__(self, camera_id: int = 0):
        super().__init__()
        self.camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
        if not self.camera.isOpened():
            raise RuntimeError(f"Could not open camera {camera_id}")

        # Low-latency camera configuration
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, CAMERA_BUFFER_SIZE)
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*DEFAULT_CODEC))
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        self._last_frame = None  # Cache last good frame
        logger.info("Camera %s initialized with low-latency settings", camera_id)

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        # Drain buffer to get the most recent frame (reduces latency)
        for _ in range(2):
            self.camera.grab()

        ret, frame = self.camera.read()
        if not ret:
            logger.warning("Failed to read frame from camera, using last frame")
            # Return last good frame instead of None to prevent stream freeze
            if self._last_frame is not None:
                video_frame = VideoFrame.from_ndarray(self._last_frame, format="rgb24")
                video_frame.pts = pts
                video_frame.time_base = time_base
                return video_frame
            # Create black frame as last resort
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
        else:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = np.ascontiguousarray(frame)
            self._last_frame = frame.copy()  # Cache good frame

        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def __del__(self):
        if hasattr(self, "camera"):
            self.camera.release()


class MultiCameraVideoTrack(VideoStreamTrack):
    """Combine three cameras: main on top, two split on bottom."""

    def __init__(self, camera_ids=None):
        super().__init__()
        self.camera_ids = self._resolve_camera_ids(camera_ids)
        self.cameras = []
        self.cam_settings: list[dict[str, float]] = []
        self.last_warn = 0.0
        self._start_time = None
        self._last_pts: int | None = None
        self._last_frame = None  # Cache last good frame to prevent freeze
        self._consecutive_failures = 0  # Track consecutive read failures
        self._stopped = False  # Track if track has been stopped
        # Single-camera mode only.
        self._layout_order = [0]
        self.single_camera_mode = True
        self.single_cam_index = 0

        # Lower defaults to reduce latency/bandwidth; override via env if needed.
        # 640x480 provides good clarity with acceptable latency
        self.width = int(os.getenv("ROBOT_CAMERA_WIDTH", "640"))
        self.height = int(os.getenv("ROBOT_CAMERA_HEIGHT", "480"))
        self.fps = float(os.getenv("ROBOT_CAMERA_FPS", "20"))

        # Many encoders (e.g., VP8/VP9 via yuv420p) require even dimensions.
        self.width = max(2, self.width - (self.width % 2))
        self.height = max(2, self.height - (self.height % 2))

        # Only use the first camera ID; single-camera mode only.
        if self.camera_ids:
            primary_id = self.camera_ids[0]
        else:
            primary_id = 0
            self.camera_ids = [primary_id]

        settings = {
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
        }
        self.cam_settings.append(settings)
        camera = self._open_camera_with_retry(primary_id, settings)
        self.cameras.append(camera)

    async def next_timestamp(self):
        if self._start_time is None:
            # Use monotonic time to avoid jumps if system clock changes.
            self._start_time = time.monotonic()

        # RTP timestamp clock for video is 90 kHz.
        pts = int((time.monotonic() - self._start_time) * 90000)

        # Ensure strictly increasing PTS (some encoders dislike repeats/backwards).
        if self._last_pts is not None and pts <= self._last_pts:
            pts = self._last_pts + 1
        self._last_pts = pts

        return pts, Fraction(1, 90000)

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        # Single-camera fast path: return the latest frame only.
        if self.single_camera_mode and self.cameras:
            cam_idx = (
                self.single_cam_index
                if self.single_cam_index < len(self.cameras)
                else 0
            )
            cam = self.cameras[cam_idx]
            frame = None
            got_frame = False

            if cam and cam.isOpened():
                # Drain buffer to get the most recent frame (reduces latency)
                # Multiple grabs ensure we skip any queued frames
                for _ in range(2):
                    cam.grab()
                ret, frame = self._read_frame(cam)
                if ret and frame is not None:
                    got_frame = True
                    self._consecutive_failures = 0  # Reset failure counter
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    frame = np.ascontiguousarray(frame)
                    if frame.shape[1] != self.width or frame.shape[0] != self.height:
                        frame = cv2.resize(frame, (self.width, self.height))
                        frame = np.ascontiguousarray(frame)
                    self._last_frame = frame.copy()  # Cache good frame
                else:
                    self._consecutive_failures += 1
            else:
                self._consecutive_failures += 1

            # Try to reopen camera if too many consecutive failures
            if (
                self._consecutive_failures >= CAMERA_REOPEN_THRESHOLD
                and not self._stopped
            ):
                logger.warning(
                    "Too many consecutive failures (%d), attempting to reopen camera",
                    self._consecutive_failures,
                )
                self._reopen_camera(cam_idx)
                self._consecutive_failures = 0

            # Use cached frame if current read failed (prevents freeze)
            if not got_frame:
                if self._last_frame is not None:
                    frame = self._last_frame
                else:
                    frame = self._create_black_frame(self.width, self.height)
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Ensure stable dimensions for encoders.
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height))
            frame = np.ascontiguousarray(frame)

            try:
                video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
            except (TypeError, ValueError):
                # If we ever end up with an invalid ndarray, fall back to a black frame.
                safe = self._create_black_frame(self.width, self.height)
                video_frame = VideoFrame.from_ndarray(safe, format="rgb24")
            video_frame.pts = pts
            video_frame.time_base = time_base
            return video_frame

        frames = []
        for idx, camera in enumerate(self.cameras):
            if camera and camera.isOpened():
                ret, frame = self._read_frame(camera)
                frames.append(
                    frame if ret else self._create_black_frame(*self._get_cam_dims(idx))
                )
            else:
                frames.append(self._create_black_frame(*self._get_cam_dims(idx)))

        composite = self._create_composite_frame(frames)
        composite = cv2.cvtColor(composite, cv2.COLOR_BGR2RGB)
        composite = np.ascontiguousarray(composite)
        self._last_frame = composite.copy()  # Cache composite frame

        try:
            video_frame = VideoFrame.from_ndarray(composite, format="rgb24")
        except (TypeError, ValueError):
            safe = self._create_black_frame(self.width, self.height)
            video_frame = VideoFrame.from_ndarray(safe, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def _create_black_frame(self, width: int | None = None, height: int | None = None):
        w = width or self.width
        h = height or self.height
        return np.zeros((h, w, 3), dtype=np.uint8)

    def _open_camera_with_retry(
        self,
        cam_id: int,
        settings: dict[str, float],
        retries: int = CAMERA_OPEN_RETRIES,
    ):
        """Open camera with retries for robustness."""
        for attempt in range(retries):
            camera = self._open_camera(cam_id, settings)
            if camera and camera.isOpened():
                # Verify we can actually read from it
                ret, _ = camera.read()
                if ret:
                    logger.info(
                        "Camera %s opened successfully on attempt %d",
                        cam_id,
                        attempt + 1,
                    )
                    return camera
                else:
                    logger.debug(
                        "Camera %s opened but can't read, retry %d/%d",
                        cam_id,
                        attempt + 1,
                        retries,
                    )
                    camera.release()
            else:
                logger.debug(
                    "Failed to open camera %s, retry %d/%d",
                    cam_id,
                    attempt + 1,
                    retries,
                )

            if attempt < retries - 1:
                time.sleep(CAMERA_OPEN_DELAY)

        _log_camera_error_throttled(
            f"camera_{cam_id}",
            "Failed to open camera %s after %d attempts (device may not exist or be in use)",
            cam_id,
            retries,
        )
        return None

    def _open_camera(self, cam_id: int, settings: dict[str, float]):
        camera = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
        if not camera.isOpened():
            logger.debug("Could not open camera %s", cam_id)
            return None
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._apply_camera_settings(camera, settings)
        logger.info("Camera %s initialized successfully", cam_id)
        return camera

    def _reopen_camera(self, cam_idx: int):
        """Attempt to reopen a camera that has stopped working."""
        if cam_idx >= len(self.cameras) or cam_idx >= len(self.camera_ids):
            return

        cam_id = self.camera_ids[cam_idx]
        settings = (
            self.cam_settings[cam_idx]
            if cam_idx < len(self.cam_settings)
            else {"width": self.width, "height": self.height, "fps": self.fps}
        )

        # Release old camera first
        old_cam = self.cameras[cam_idx]
        if old_cam:
            try:
                old_cam.release()
            except Exception as e:
                logger.debug("Error releasing old camera: %s", e)

        # Small delay to allow device to be freed
        time.sleep(0.2)

        # Try to reopen
        new_cam = self._open_camera_with_retry(cam_id, settings)
        self.cameras[cam_idx] = new_cam

        if new_cam:
            logger.info("Camera %s (index %d) reopened successfully", cam_id, cam_idx)
        else:
            logger.error("Failed to reopen camera %s (index %d)", cam_id, cam_idx)

    def _read_frame(self, camera):
        try:
            grabbed = camera.grab()
            if not grabbed:
                return False, None
            ret, frame = camera.retrieve()
            if not ret:
                return False, None
            return True, frame
        except Exception as exc:  # pragma: no cover
            now = time.time()
            if now - self.last_warn > 5:
                logger.warning("Camera read failed: %s", exc)
                self.last_warn = now
            return False, None

    def _create_composite_frame(self, frames):
        while len(frames) < 3:
            frames.append(self._create_black_frame())

        # Apply current layout order; pad with black frames if indexes are out of range.
        ordered = []
        for idx in self._layout_order:
            if 0 <= idx < len(frames):
                ordered.append(frames[idx])
            else:
                ordered.append(self._create_black_frame(*self._get_cam_dims(idx)))

        # Derive native frame size to preserve aspect ratio; fall back to configured defaults.
        ref = next((f for f in ordered if f is not None), None)
        base_h, base_w = (
            (ref.shape[0], ref.shape[1])
            if ref is not None
            else (self.height, self.width)
        )

        # Ensure even dimensions (yuv420p requirement) and non-zero.
        base_w = max(2, int(base_w) - (int(base_w) % 2))
        base_h = max(2, int(base_h) - (int(base_h) % 2))

        # Preserve aspect by scaling heights alongside widths.
        main_w = 2 * base_w
        main_h = int(main_w * (base_h / base_w))
        main_h = max(2, main_h - (main_h % 2))
        main_frame = cv2.resize(ordered[0], (main_w, main_h))

        bottom_left = cv2.resize(ordered[1], (base_w, base_h))
        bottom_right = cv2.resize(ordered[2], (base_w, base_h))

        bottom_combined = np.hstack([bottom_left, bottom_right])
        composite = np.vstack([main_frame, bottom_combined])
        return composite

    def __del__(self):
        self.stop()

    def _resolve_camera_ids(self, camera_ids):
        # Priority: explicit argument -> env override -> auto-detect.
        if camera_ids:
            return camera_ids

        env_ids_raw = os.getenv("ROBOT_CAMERA_IDS")
        if env_ids_raw:
            try:
                env_ids = [int(x) for x in env_ids_raw.split(",") if x.strip()]
                if env_ids:
                    logger.info("Using camera IDs from ROBOT_CAMERA_IDS: %s", env_ids)
                    return env_ids
            except ValueError:
                logger.warning(
                    "Invalid ROBOT_CAMERA_IDS value '%s', ignoring", env_ids_raw
                )

        auto_ids = self._auto_detect_cameras(
            max_cameras=int(os.getenv("ROBOT_CAMERA_SCAN_MAX", "8")),
            max_needed=3,
        )
        if auto_ids:
            logger.info("Auto-detected cameras: %s", auto_ids)
            return auto_ids

        logger.error("No cameras detected; falling back to placeholders")
        return []

    def _auto_detect_cameras(self, max_cameras: int = 8, max_needed: int = 3):
        found = []
        for cam_id in range(max_cameras):
            cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
            if not cap.isOpened():
                continue
            ok, _ = cap.read()
            cap.release()
            if ok:
                found.append(cam_id)
                if len(found) >= max_needed:
                    break
        return found

    def swap_primary_secondary(self):
        # Swap the first two layout slots (main and bottom-left).
        self._layout_order[0], self._layout_order[1] = (
            self._layout_order[1],
            self._layout_order[0],
        )
        # Swap per-slot settings to keep dimensions aligned with slots.
        if len(self.cam_settings) >= 2:
            self.cam_settings[0], self.cam_settings[1] = (
                self.cam_settings[1],
                self.cam_settings[0],
            )
        logger.info("Swapped camera layout order to %s", self._layout_order)

    def cycle_layout(self):
        # Rotate layout so each camera can appear in every slot over successive calls.
        if not self._layout_order:
            return
        self._layout_order = self._layout_order[1:] + self._layout_order[:1]
        logger.info("Cycled camera layout order to %s", self._layout_order)

    def set_layout(self, order: list[int]):
        """Set layout order explicitly: [main, bottom_left, bottom_right]."""
        if not order or len(order) != 3:
            raise ValueError("layout_order must be a list of three camera indices")
        # Accept any int; frames are padded with black if out of range.
        self._layout_order = [int(x) for x in order]
        logger.info("Layout order updated to %s", self._layout_order)

    def set_camera_config(self, camera_index: int, width: int, height: int, fps: float):
        """Switch to a specific camera index and apply resolution/fps.

        Releases any existing camera handles and opens only the requested camera.
        """
        self.single_camera_mode = True
        self.single_cam_index = 0
        self.camera_ids = [int(camera_index)]
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self.cam_settings = [
            {"width": self.width, "height": self.height, "fps": self.fps}
        ]
        self._consecutive_failures = 0  # Reset failure counter

        # Release old cameras
        for cam in self.cameras:
            if cam:
                try:
                    cam.release()
                except Exception:
                    pass
        self.cameras = []

        # Small delay to allow device to be freed
        time.sleep(0.2)

        cam = self._open_camera_with_retry(self.camera_ids[0], self.cam_settings[0])
        self.cameras.append(cam)
        if cam:
            logger.info(
                "Camera switched to index %s at %sx%s@%sfps",
                self.camera_ids[0],
                self.width,
                self.height,
                self.fps,
            )
        else:
            _log_camera_error_throttled(
                f"camera_{self.camera_ids[0]}",
                "Failed to open camera %s (device may not exist or be in use)",
                self.camera_ids[0],
            )

    def stop(self):
        """Stop the video track and release all cameras."""
        self._stopped = True
        for camera in self.cameras:
            if camera:
                try:
                    camera.release()
                except Exception:
                    pass
        self.cameras = []
        logger.info("Video track stopped")

    def _get_cam_dims(self, idx: int) -> tuple[int, int]:
        if 0 <= idx < len(self.cam_settings):
            s = self.cam_settings[idx]
            return int(s.get("width", self.width)), int(s.get("height", self.height))
        return self.width, self.height

    def _apply_camera_settings(self, camera, settings: dict):
        try:
            if "width" in settings:
                camera.set(cv2.CAP_PROP_FRAME_WIDTH, settings["width"])
            if "height" in settings:
                camera.set(cv2.CAP_PROP_FRAME_HEIGHT, settings["height"])
            if "fps" in settings:
                camera.set(cv2.CAP_PROP_FPS, settings["fps"])
        except Exception as exc:
            logger.warning("Failed to apply camera settings: %s", exc)

    def apply_settings(self, camera_settings: list[dict]):
        """Update per-camera width/height/fps by slot index (0-based)."""
        if not camera_settings:
            return
        for cfg in camera_settings:
            try:
                idx = int(cfg.get("slot", -1))
            except Exception:
                continue
            if idx < 0 or idx >= len(self.cameras):
                continue
            width = cfg.get("width")
            height = cfg.get("height")
            fps = cfg.get("fps")
            if idx >= len(self.cam_settings):
                self.cam_settings.append(
                    {"width": self.width, "height": self.height, "fps": self.fps}
                )
            if width:
                self.cam_settings[idx]["width"] = float(width)
            if height:
                self.cam_settings[idx]["height"] = float(height)
            if fps:
                self.cam_settings[idx]["fps"] = float(fps)

            camera = self.cameras[idx]
            if camera and camera.isOpened():
                self._apply_camera_settings(camera, self.cam_settings[idx])
                logger.info(
                    "Updated camera %s settings to %s", idx, self.cam_settings[idx]
                )
