# Cloud signaling/relay server (Dockerized)

## Build the image

```bash
docker build -t teleop-cloud-server .
```

## Run the container

```bash
docker run --rm -p 8080:8080 -e PORT=8080 teleop-cloud-server
```

## Environment

- `PORT` / `CLOUD_SERVER_PORT`: port the server listens on (default `8080`).
- `HOST`: bind address (default `0.0.0.0`).

## Run on the robot (publisher)

From the repo root on the robot:

```bash
python -m robot.cloud_publisher
```

Env vars:

- `CLOUD_URL`: cloud server base URL (e.g., `http://<public-ip>:8080`).
- `ROBOT_CAMERA_IDS`: comma-separated camera indices (default `0,2,4`).
- `ROBOT_AUDIO_INPUT_DEVICE` / `ROBOT_AUDIO_INPUT_FORMAT`: audio capture device/format for mic.
- `ROBOT_AUDIO_OUTPUT_DEVICE` / `ROBOT_AUDIO_OUTPUT_FORMAT`: audio playback device/format for speakers.
- `ROBOT_DISABLE_AUDIO`: set to `1` to skip audio setup if ALSA/Pulse devices are not available.

Control messages (DataChannel `control` from client → robot):

- `{"head": {"pan": <deg>, "tilt": <deg>}}`
- `{"base": {"x": <m/s>, "y": <m/s>, "theta": <deg/s>}}`

## API endpoints

- `POST /robot_offer`: Robot posts its WebRTC offer; server replies with answer and fans out its tracks.
- `POST /client_offer`: Viewer posts its offer; server replies with answer, attaches robot media, and forwards `control` DataChannel messages back to the robot.
