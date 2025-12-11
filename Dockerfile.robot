# Robot client image for Raspberry Pi
# Build (example for arm/v7):
#   docker build --platform linux/arm/v7 -f Dockerfile.robot -t teleop-robot .
# Run (mount /dev/video* etc. as needed):
#   docker run --rm \
#     --device /dev/video0 \
#     --device /dev/video1 \
#     --device /dev/video2 \
#     -e CLOUD_SERVER_URL=http://<server>:8080 \
#     -e ROBOT_CAMERA_IDS=0,2,4 \
#     teleop-robot

FROM --platform=$TARGETPLATFORM python:3.10-slim-bullseye

ENV PYTHONDONTWRITEBYTECODE=1 \
  PYTHONUNBUFFERED=1 \
  PIP_NO_CACHE_DIR=1

# System deps for camera + aiortc/av/opencv
RUN apt-get update \ 
  && apt-get install -y --no-install-recommends \ 
  ffmpeg \ 
  v4l-utils \ 
  libsm6 \ 
  libxext6 \ 
  libgl1 \ 
  libglib2.0-0 \ 
  && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY pyproject.toml uv.lock ./
RUN pip install --upgrade pip setuptools wheel \ 
  && pip install --no-cache-dir \ 
  aiohttp>=3.13.2 \ 
  aiortc>=1.14.0 \ 
  "av>=15.0.0,<=16.0.0" \ 
  cv2-enumerate-cameras>=1.3.1 \ 
  "lerobot[all]<=0.4.0" \ 
  opencv-python>=4.12.0.88

COPY src ./src

# Default environment (override at runtime)
ENV CLOUD_SERVER_URL=http://localhost:8080 \
  ROBOT_CAMERA_IDS=0,2,4

CMD ["python", "-m", "robot.client"]
