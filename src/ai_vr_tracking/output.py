from __future__ import annotations

import json
import socket
import struct
from typing import Iterable, List

from .models import FusedFrame, OutputConfig


def _pad_osc_string(value: str) -> bytes:
    encoded = value.encode("utf-8") + b"\x00"
    padding = (4 - (len(encoded) % 4)) % 4
    return encoded + (b"\x00" * padding)


def _encode_osc_message(address: str, arguments: Iterable[object]) -> bytes:
    tags = [","]
    payload = bytearray()
    for argument in arguments:
        if isinstance(argument, str):
            tags.append("s")
            payload.extend(_pad_osc_string(argument))
        elif isinstance(argument, int):
            tags.append("i")
            payload.extend(struct.pack(">i", argument))
        else:
            tags.append("f")
            payload.extend(struct.pack(">f", float(argument)))
    return b"".join([_pad_osc_string(address), _pad_osc_string("".join(tags)), bytes(payload)])


class OscOutputSink:
    def __init__(self, host: str, port: int, emit_joints: bool = True) -> None:
        self._host = host
        self._port = port
        self._emit_joints = emit_joints
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, frame: FusedFrame) -> None:
        packets: List[bytes] = [
            _encode_osc_message(
                "/tracking/meta",
                [frame.timestamp, len(frame.active_cameras), len(frame.trackers)],
            )
        ]
        for name, tracker in frame.trackers.items():
            packets.append(
                _encode_osc_message(
                    f"/tracking/tracker/{name}",
                    [tracker.x, tracker.y, tracker.z, tracker.confidence],
                )
            )

        if self._emit_joints:
            for name, joint in frame.joints.items():
                packets.append(
                    _encode_osc_message(
                        f"/tracking/joint/{name}",
                        [joint.x, joint.y, joint.z, joint.visibility],
                    )
                )

        for packet in packets:
            self._socket.sendto(packet, (self._host, self._port))

    def close(self) -> None:
        self._socket.close()


class JsonUdpOutputSink:
    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, frame: FusedFrame) -> None:
        payload = json.dumps(frame.to_dict(), separators=(",", ":")).encode("utf-8")
        self._socket.sendto(payload, (self._host, self._port))

    def close(self) -> None:
        self._socket.close()


class DirectVmtOutputSink:
    def __init__(self, host: str, port: int) -> None:
        from .vmt_bridge import BridgeConfig, VmtBridgeRuntime

        self._runtime = VmtBridgeRuntime(BridgeConfig(vmt_host=host, vmt_port=port))

    def send(self, frame: FusedFrame) -> None:
        self._runtime.process_packet(frame.to_dict())

    def close(self) -> None:
        self._runtime.close()


class TrackingOutputRouter:
    def __init__(self, config: OutputConfig) -> None:
        self._config = config
        self._sink = None
        self._build_sink()

    def _build_sink(self) -> None:
        if not self._config.enabled:
            self._sink = None
            return

        protocol = self._config.protocol.lower().strip()
        if protocol == "json_udp":
            self._sink = JsonUdpOutputSink(self._config.host, self._config.port)
        elif protocol == "vmt_osc":
            self._sink = DirectVmtOutputSink(self._config.host, self._config.port)
        else:
            self._sink = OscOutputSink(self._config.host, self._config.port, self._config.emit_joints)

    def update_config(self, config: OutputConfig) -> None:
        self.close()
        self._config = config
        self._build_sink()

    def send(self, frame: FusedFrame) -> None:
        if self._sink is None:
            return
        self._sink.send(frame)

    def close(self) -> None:
        if self._sink is not None:
            self._sink.close()
            self._sink = None
