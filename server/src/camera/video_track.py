import cv2
import logging
import numpy as np
import os
import time
from aiortc import VideoStreamTrack
from av import VideoFrame

logger = logging.getLogger(__name__)


class CameraVideoTrack(VideoStreamTrack):
    """Single-camera video track using OpenCV."""

    def __init__(self, camera_id: int = 0):
        super().__init__()
        self.camera = cv2.VideoCapture(camera_id)
        if not self.camera.isOpened():
            raise RuntimeError(f"Could not open camera {camera_id}")

        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        logger.info("Camera %s initialized successfully", camera_id)

    async def recv(self):
        pts, time_base = await self.next_timestamp()
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
        self.camera_ids = camera_ids or [0, 2, 4]
        self.cameras = []
        self.last_warn = 0.0
        self._start_time = None

        # Lower defaults to reduce latency/bandwidth; override via env if needed.
        self.width = int(os.getenv("ROBOT_CAMERA_WIDTH", "640"))
        self.height = int(os.getenv("ROBOT_CAMERA_HEIGHT", "360"))
        self.fps = float(os.getenv("ROBOT_CAMERA_FPS", "15"))

        for cam_id in self.camera_ids:
            camera = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
            if not camera.isOpened():
                logger.warning("Could not open camera %s", cam_id)
                self.cameras.append(None)
                continue

            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            camera.set(cv2.CAP_PROP_FPS, self.fps)
            self.cameras.append(camera)
            logger.info("Camera %s initialized successfully", cam_id)

    async def next_timestamp(self):
        if self._start_time is None:
            self._start_time = time.time()
        
        # Use wall clock time for PTS to sync with audio
        pts = int((time.time() - self._start_time) * 90000)
        return pts, 90000

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        frames = []
        for camera in self.cameras:
            if camera and camera.isOpened():
                ret, frame = self._read_frame(camera)
                frames.append(frame if ret else self._create_black_frame())
            else:
                frames.append(self._create_black_frame())

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

        main_frame = cv2.resize(frames[0], (self.width * 2, self.height))
        bottom_left = cv2.resize(frames[1], (self.width, self.height))
        bottom_right = cv2.resize(frames[2], (self.width, self.height))

        bottom_combined = np.hstack([bottom_left, bottom_right])
        composite = np.vstack([main_frame, bottom_combined])
        return composite

    def __del__(self):
        for camera in self.cameras:
            if camera:
                camera.release()
