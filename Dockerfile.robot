# syntax=docker/dockerfile:1

FROM python:3.10-slim AS runtime

ENV PYTHONUNBUFFERED=1 \
  PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPATH=/app \
  CLOUD_SERVER_URL=http://teleop.loopcow.com:8080

# System packages required by aiortc/av, OpenCV, lerobot and camera access
RUN apt-get update && apt-get install -y --no-install-recommends \
  ffmpeg \
  libsm6 \
  libxext6 \
  libgl1 \
  libglib2.0-0 \
  libsrtp2-1 \
  v4l-utils \
  libasound2 \
  alsa-utils \
  git \
  build-essential \
  libhdf5-dev \
  libusb-1.0-0 \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy pyproject.toml first for dependency caching
COPY pyproject.toml /app/

RUN pip install --no-cache-dir --upgrade pip
# Install all project dependencies including lerobot
RUN pip install --no-cache-dir \
  aiohttp>=3.13.2 \
  aiortc>=1.14.0 \
  "av>=15.0.0,<=16.0.0" \
  opencv-python-headless \
  numpy \
  "lerobot[all]<=0.4.0"

# Copy robot client sources
COPY src/robot /app/src/robot
COPY src/camera /app/src/camera

CMD ["python", "-m", "src.robot.client"]
