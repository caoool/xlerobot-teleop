# Cloud signaling/relay server

## Local TLS quick start (port 443)

```bash
./scripts/generate_dev_cert.sh           # writes certs/teleop-dev.{crt,key}
SSL_CERT_FILE=certs/teleop-dev.crt \
SSL_KEY_FILE=certs/teleop-dev.key \
PORT=443 uv run src/server/server.py
```

## Build the image

```bash
docker build -t teleop-cloud-server .
```

## Run the container (HTTPS on 443)

```bash
docker run --rm \
	-p 443:443 \
	-e PORT=443 \
	-e SSL_CERT_FILE=/certs/teleop-dev.crt \
	-e SSL_KEY_FILE=/certs/teleop-dev.key \
	-v $(pwd)/certs:/certs:ro \
	teleop-cloud-server
```

## Environment

- `PORT` / `CLOUD_SERVER_PORT`: port to listen on (default `443`).
- `HOST`: bind address (default `0.0.0.0`).
- `SSL_CERT_FILE` / `SSL_KEY_FILE`: paths to TLS cert and key; enable HTTPS when both are set.
- `SSL_PASSWORD`: optional password for encrypted key.

## Run on the robot (publisher)

```bash
CLOUD_SERVER_URL="https://<server-host>" uv run src/robot/client.py
```

Useful env vars:

- `ROBOT_CAMERA_IDS`: comma-separated camera indices (default `0,2,4`).
- `ROBOT_ICE_SERVERS`: JSON list of ICE servers; defaults to the bundled TURN/STUN.
- `ROBOT_AUDIO_INPUT_DEVICE` / `ROBOT_AUDIO_INPUT_FORMAT`: audio capture device/format for mic.
- `ROBOT_AUDIO_OUTPUT_DEVICE` / `ROBOT_AUDIO_OUTPUT_FORMAT`: audio playback device/format for speakers.
- `ROBOT_DISABLE_AUDIO`: set to `1` to skip audio setup if ALSA/Pulse devices are not available.

## API endpoints

- `POST /robot/offer`: Robot posts its WebRTC offer; server replies with answer and fans out its tracks.
- `POST /offer`: Viewer posts its offer; server replies with answer, attaches robot media, and forwards `control` DataChannel messages back to the robot.
