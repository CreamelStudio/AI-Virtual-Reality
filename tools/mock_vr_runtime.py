#!/usr/bin/env python3

import argparse
import json
import socket
from typing import Dict


def format_trackers(trackers: Dict[str, Dict[str, float]]) -> str:
    pieces = []
    for name in sorted(trackers):
        tracker = trackers[name]
        pieces.append(
            f"{name}=({tracker['x']:.2f}, {tracker['y']:.2f}, {tracker['z']:.2f}) c={tracker['confidence']:.2f}"
        )
    return " | ".join(pieces)


def main() -> None:
    parser = argparse.ArgumentParser(description="Simple JSON/UDP listener for AI VR tracking output.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7000)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))

    print(f"Listening on udp://{args.host}:{args.port}")
    while True:
        payload, address = sock.recvfrom(65535)
        try:
            packet = json.loads(payload.decode("utf-8"))
        except json.JSONDecodeError:
            print(f"{address}: invalid JSON packet")
            continue

        trackers = packet.get("trackers", {})
        timestamp = packet.get("timestamp", 0.0)
        mode = packet.get("mode", "unknown")
        active_cameras = packet.get("active_cameras", [])
        print(
            f"{address} t={timestamp:.2f} mode={mode} cams={active_cameras} "
            f"{format_trackers(trackers)}"
        )


if __name__ == "__main__":
    main()

