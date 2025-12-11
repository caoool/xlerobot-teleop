import cv2
import logging
import numpy as np
import os
import time
from aiortc import VideoStreamTrack
from av import VideoFrame

logger = logging.getLogger(__name__)

# Low-latency tuning constants
CAMERA_BUFFER_SIZE = 1  # Minimum buffer to get latest frame
DEFAULT_CODEC = "MJPG"  # Hardware-accelerated, low latency


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
        logger.info("Camera %s initialized with low-latency settings", camera_id)

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        # Drain buffer to get the most recent frame (reduces latency)
        for _ in range(2):
            self.camera.grab()

        ret, frame = self.camera.read()
        if not ret:
            logger.error("Failed to read frame from camera")
            return None

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
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
        # Single-camera mode only.
        self._layout_order = [0]
        self.single_camera_mode = True
        self.single_cam_index = 0

        # Lower defaults to reduce latency/bandwidth; override via env if needed.
        # 640x480 provides good clarity with acceptable latency
        self.width = int(os.getenv("ROBOT_CAMERA_WIDTH", "640"))
        self.height = int(os.getenv("ROBOT_CAMERA_HEIGHT", "480"))
        self.fps = float(os.getenv("ROBOT_CAMERA_FPS", "20"))

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
        camera = self._open_camera(primary_id, settings)
        self.cameras.append(camera)

    async def next_timestamp(self):
        if self._start_time is None:
            self._start_time = time.time()

        # Use wall clock time for PTS to sync with audio
        pts = int((time.time() - self._start_time) * 90000)
        return pts, 90000

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
            if cam and cam.isOpened():
                # Drain buffer to get the most recent frame (reduces latency)
                # Multiple grabs ensure we skip any queued frames
                for _ in range(2):
                    cam.grab()
                ret, frame = self._read_frame(cam)
                if not ret:
                    frame = self._create_black_frame(self.width, self.height)
            else:
                frame = self._create_black_frame(self.width, self.height)

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
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

        video_frame = VideoFrame.from_ndarray(composite, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def _create_black_frame(self, width: int | None = None, height: int | None = None):
        w = width or self.width
        h = height or self.height
        return np.zeros((h, w, 3), dtype=np.uint8)

    def _open_camera(self, cam_id: int, settings: dict[str, float]):
        camera = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
        if not camera.isOpened():
            logger.warning("Could not open camera %s", cam_id)
            return None
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._apply_camera_settings(camera, settings)
        logger.info("Camera %s initialized successfully", cam_id)
        return camera

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

        # Preserve aspect by scaling heights alongside widths.
        main_w = 2 * base_w
        main_h = int(main_w * (base_h / base_w))
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

        # Release old cameras
        for cam in self.cameras:
            if cam:
                try:
                    cam.release()
                except Exception:
                    pass
        self.cameras = []

        cam = self._open_camera(self.camera_ids[0], self.cam_settings[0])
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
            logger.error("Failed to open camera %s", self.camera_ids[0])

    def stop(self):
        for camera in self.cameras:
            if camera:
                try:
                    camera.release()
                except Exception:
                    pass

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
