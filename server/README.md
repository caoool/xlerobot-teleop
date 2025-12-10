Cloud signaling/relay server (Dockerized)
========================================

Build the image
---------------

```bash
docker build -t teleop-cloud-server .
```

Run the container
-----------------

```bash
docker run --rm -p 8080:8080 -e PORT=8080 teleop-cloud-server
```

Environment
-----------

- `PORT` / `CLOUD_SERVER_PORT`: port the server listens on (default `8080`).
- `HOST`: bind address (default `0.0.0.0`).

API endpoints
-------------

- `POST /robot_offer`: Robot posts its WebRTC offer; server replies with answer and fans out its tracks.
- `POST /client_offer`: Viewer posts its offer; server replies with answer, attaches robot media, and forwards `control` DataChannel messages back to the robot.
