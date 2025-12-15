# Remote XLeRobot Teleop 🤖🌍

WARNING: AI generated README.md, no guarantee it would run, physical parts (robot, board, speark ...) will be updated (model, where to buy) soon, real performance videos will be uploaded soon.

Welcome! This repo is a **remote teleoperation stack** for a robot built from the awesome
[XLeRobot](https://github.com/Vector-Wangel/XLeRobot) project.

The goal is simple: **be “present” with a physical robot when you’re not there** — for family time, quick check-ins, or remote work meetings.

This project is heavily AI-assisted (and proudly so). ✨

## What you get ✅

- 🎥 **WebRTC video** streaming from the robot
- 🔊 **Two-way audio** (optional; can be disabled)
- 🕹️ **Web UI** for teleop + chat-like interaction
- 🔐 **Login page** for basic authentication
- 🧠 **Control data channel** (head + base controls today)

## Quick start (local dev) 🚀

### 1) Run the cloud signaling/relay server

```bash
uv run src/server/server.py
```

By default it listens on `http://0.0.0.0:8080`.

### 2) Run the robot client (publisher)

```bash
CLOUD_SERVER_URL="http://<server-host>:8080" \
uv run src/robot/client.py
```

### 3) Open the Web UI

Open `http://<server-host>:8080` in your browser.

## Run the robot client with Docker 🐳

This is useful on the robot computer (camera + hardware access).

```bash
docker compose -f compose.robot.yml up --build
```

Common environment variables:

- `CLOUD_SERVER_URL`: where the robot should connect (e.g. `http://<server-host>:8080`)
- `ROBOT_CAMERA_IDS`: comma-separated camera indices (default `0`)
- `ROBOT_DISABLE_AUDIO=1`: disable audio if ALSA devices are missing
- `ROBOT_ICE_SERVERS`: JSON list of ICE servers (optional)

## Joy-Con arm control (In Progress)🎮🦾

Joy-Con support is implemented in the Web UI (WebHID). A **deadman switch** is required:

- Hold **LT + RT** to enable all Joy-Con actions (Joy-Con: `ZL+ZR`, also accepts `L+R`).
- When not held, **all arm/gripper motion is ignored**.

## Roadmap 🗺️

- [x] WebRTC for two-way communication (video & audio)
- [x] Web UI for controlling the robot + communication
- [x] Simple login page
- [x] Head movement + base movement
- [ ] Joy-Con controller to control the robot arms
- [ ] Refined control (latency + smoothing)
- [ ] Face tracking for expression mapping
- [ ] 360 camera for VR integration
- [ ] Bare-hand control
- [ ] Human-like android for better emotional presence
- [ ] Auto sub-tasks driven by AI

## Contributing / feedback 🙌

If you’re interested, have ideas, or want to collaborate on real robotics projects, please reach out!

## Disclaimer (responsibility-free) ⚠️

This project is provided **as-is**, without any warranties or guarantees.

- 🧑‍🔧 **Use at your own risk**: you are responsible for safe setup, testing, and operation.
- 🛑 **Safety first**: always have a physical emergency stop and test at low speed/low power.
- 🧯 **Real hardware can be dangerous**: keep clear of moving parts and never leave the robot unattended while enabled.
- 🧩 **Your environment is unique**: network conditions, cameras, motors, and drivers vary; expect to tune and validate.

## Thanks / Acknowledgements 💙

Big thanks to all the great open-source projects that make this possible:

- [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) — the robot design and ecosystem that inspired this repo
- [aiortc](https://github.com/aiortc/aiortc) — WebRTC implementation in Python
- [aiohttp](https://github.com/aio-libs/aiohttp) — async web server and HTTP client
- [coturn](https://github.com/coturn/coturn) — TURN/STUN server for NAT traversal
- [LeRobot](https://github.com/huggingface/lerobot) — robotics utilities and integrations

And thanks to everyone sharing knowledge in the robotics + WebRTC communities. 🙏
