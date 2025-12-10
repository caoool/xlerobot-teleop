"""Entrypoint for running the multi-camera WebRTC server."""

import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "src"))

from server import run_server


DEFAULT_CAMERA_IDS = (0, 2, 4)


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    run_server(host="0.0.0.0", port=8080, camera_ids=DEFAULT_CAMERA_IDS)


if __name__ == "__main__":
    main()
