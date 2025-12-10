FROM python:3.10-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/app/src

# System deps for aiortc/av build and runtime
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libavdevice-dev libavfilter-dev libavformat-dev libavcodec-dev libavutil-dev \
    libswresample-dev libswscale-dev \
    libopus-dev libvpx-dev pkg-config libssl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy only what we need to run the cloud server
COPY pyproject.toml uv.lock ./
COPY src ./src

# Install Python deps (minimal set needed for cloud relay)
RUN python -m pip install --no-cache-dir --upgrade pip && \
    python -m pip install --no-cache-dir aiohttp>=3.13.2 aiortc>=1.14.0

EXPOSE 8080

CMD ["python", "-m", "server.cloud_server"]
