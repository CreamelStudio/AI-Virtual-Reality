from __future__ import annotations

import argparse
import ctypes
import json
import math
import socket
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Tuple

from .output import _encode_osc_message


EPSILON = 1e-6
HMD_LOCAL_XZ_BLEND = 0.25
HMD_LOCAL_Y_BLEND = 0.60


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vec3":
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    def length(self) -> float:
        return math.sqrt((self.x * self.x) + (self.y * self.y) + (self.z * self.z))

    def normalized(self, fallback: "Vec3" | None = None) -> "Vec3":
        magnitude = self.length()
        if magnitude <= EPSILON:
            return fallback or Vec3(0.0, 1.0, 0.0)
        return self * (1.0 / magnitude)

    def rotate_y(self, radians: float) -> "Vec3":
        cosine = math.cos(radians)
        sine = math.sin(radians)
        return Vec3(
            (self.x * cosine) - (self.z * sine),
            self.y,
            (self.x * sine) + (self.z * cosine),
        )

    @staticmethod
    def cross(left: "Vec3", right: "Vec3") -> "Vec3":
        return Vec3(
            (left.y * right.z) - (left.z * right.y),
            (left.z * right.x) - (left.x * right.z),
            (left.x * right.y) - (left.y * right.x),
        )


@dataclass(frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float

    def normalized(self) -> "Quaternion":
        magnitude = math.sqrt((self.x * self.x) + (self.y * self.y) + (self.z * self.z) + (self.w * self.w))
        if magnitude <= EPSILON:
            return Quaternion(0.0, 0.0, 0.0, 1.0)
        return Quaternion(
            self.x / magnitude,
            self.y / magnitude,
            self.z / magnitude,
            self.w / magnitude,
        )


@dataclass(frozen=True)
class DeviceBinding:
    index: int
    enable_mode: int = 7


@dataclass(frozen=True)
class ControllerBinding:
    name: str
    side: str
    index: int
    enable_mode: int


@dataclass
class BridgeConfig:
    listen_host: str = "0.0.0.0"
    listen_port: int = 7000
    vmt_host: str = "127.0.0.1"
    vmt_port: int = 39570
    hmd_host: str = "127.0.0.1"
    hmd_port: int = 39575
    enable_hmd: bool = True
    enable_controllers: bool = True
    scale: float = 1.0
    x_axis_scale: float = 1.0
    y_axis_scale: float = 1.0
    z_axis_scale: float = -1.0
    hmd_y_offset: float = 0.0
    controller_position_scale: float = 2.2
    waist_height: float = 0.95
    foot_floor_offset: float = 0.02
    packet_timeout: float = 1.5
    print_every: float = 1.0
    bindings: Dict[str, DeviceBinding] = field(
        default_factory=lambda: {
            "waist": DeviceBinding(index=0),
            "chest": DeviceBinding(index=1),
            "head": DeviceBinding(index=2),
            "left_hand": DeviceBinding(index=3),
            "right_hand": DeviceBinding(index=4),
            "left_foot": DeviceBinding(index=5),
            "right_foot": DeviceBinding(index=6),
            "left_knee": DeviceBinding(index=7),
            "right_knee": DeviceBinding(index=8),
            "left_elbow": DeviceBinding(index=9),
            "right_elbow": DeviceBinding(index=10),
        }
    )
    controller_bindings: Dict[str, ControllerBinding] = field(
        default_factory=lambda: {
            "left_controller": ControllerBinding(name="left_controller", side="left", index=20, enable_mode=5),
            "right_controller": ControllerBinding(name="right_controller", side="right", index=21, enable_mode=6),
        }
    )


@dataclass
class CalibrationState:
    ready: bool = False
    origin: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))
    yaw_correction: float = 0.0
    y_offset: float = 0.95
    hmd_origin: Vec3 | None = None
    hmd_reference: Vec3 | None = None
    hmd_root_origin: Vec3 | None = None
    hmd_root_offset: Vec3 | None = None

    def transform(self, vector: Vec3, config: BridgeConfig) -> Vec3:
        translated = vector - self.origin
        rotated = translated.rotate_y(self.yaw_correction)
        scaled = rotated * config.scale
        return Vec3(scaled.x, scaled.y + self.y_offset, scaled.z)

    def transform_delta(self, vector: Vec3, config: BridgeConfig) -> Vec3:
        rotated = vector.rotate_y(self.yaw_correction)
        return rotated * config.scale


@dataclass(frozen=True)
class VmtPose:
    position: Vec3
    rotation: Quaternion


def _average(vectors: Iterable[Vec3]) -> Vec3 | None:
    values = list(vectors)
    if not values:
        return None
    count = float(len(values))
    return Vec3(
        sum(value.x for value in values) / count,
        sum(value.y for value in values) / count,
        sum(value.z for value in values) / count,
    )


def _pick_center(mapping: Dict[str, Vec3], names: Iterable[str]) -> Vec3 | None:
    return _average(mapping[name] for name in names if name in mapping)


def _distance(left: Vec3, right: Vec3) -> float:
    return (left - right).length()


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _apply_deadzone(value: float, deadzone: float = 0.2) -> float:
    if abs(value) <= deadzone:
        return 0.0
    scaled = (abs(value) - deadzone) / max(1.0 - deadzone, EPSILON)
    return math.copysign(_clamp(scaled, 0.0, 1.0), value)


def _scale_axes(vector: Vec3, config: BridgeConfig) -> Vec3:
    return Vec3(
        vector.x * config.x_axis_scale,
        vector.y * config.y_axis_scale,
        vector.z * config.z_axis_scale,
    )


def _parse_vec3(payload: Dict[str, float]) -> Vec3:
    return Vec3(float(payload["x"]), float(payload["y"]), float(payload["z"]))


def _build_basis(
    *,
    right: Vec3 | None = None,
    up: Vec3 | None = None,
    forward: Vec3 | None = None,
    fallback_forward: Vec3 | None = None,
) -> Tuple[Vec3, Vec3, Vec3]:
    fallback = (fallback_forward or Vec3(0.0, 0.0, 1.0)).normalized(Vec3(0.0, 0.0, 1.0))

    if right is not None and up is not None:
        right_axis = right.normalized(Vec3(1.0, 0.0, 0.0))
        up_axis = up.normalized(Vec3(0.0, 1.0, 0.0))
        forward_axis = Vec3.cross(right_axis, up_axis).normalized(fallback)
        up_axis = Vec3.cross(forward_axis, right_axis).normalized(Vec3(0.0, 1.0, 0.0))
        return right_axis, up_axis, forward_axis

    if up is not None and forward is not None:
        up_axis = up.normalized(Vec3(0.0, 1.0, 0.0))
        forward_axis = forward.normalized(fallback)
        right_axis = Vec3.cross(up_axis, forward_axis).normalized(Vec3(1.0, 0.0, 0.0))
        forward_axis = Vec3.cross(right_axis, up_axis).normalized(fallback)
        return right_axis, up_axis, forward_axis

    if right is not None and forward is not None:
        right_axis = right.normalized(Vec3(1.0, 0.0, 0.0))
        forward_axis = forward.normalized(fallback)
        up_axis = Vec3.cross(forward_axis, right_axis).normalized(Vec3(0.0, 1.0, 0.0))
        forward_axis = Vec3.cross(right_axis, up_axis).normalized(fallback)
        return right_axis, up_axis, forward_axis

    return Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0), fallback


def _quaternion_from_basis(right: Vec3, up: Vec3, forward: Vec3) -> Quaternion:
    m00, m01, m02 = right.x, up.x, forward.x
    m10, m11, m12 = right.y, up.y, forward.y
    m20, m21, m22 = right.z, up.z, forward.z
    trace = m00 + m11 + m22

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = Quaternion(
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
            0.25 * scale,
        )
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        quaternion = Quaternion(
            0.25 * scale,
            (m01 + m10) / scale,
            (m02 + m20) / scale,
            (m21 - m12) / scale,
        )
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        quaternion = Quaternion(
            (m01 + m10) / scale,
            0.25 * scale,
            (m12 + m21) / scale,
            (m02 - m20) / scale,
        )
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        quaternion = Quaternion(
            (m02 + m20) / scale,
            (m12 + m21) / scale,
            0.25 * scale,
            (m10 - m01) / scale,
        )

    return quaternion.normalized()


def _torso_forward(joints: Dict[str, Vec3]) -> Vec3:
    shoulders = _pick_center(joints, ["left_shoulder", "right_shoulder"])
    hips = _pick_center(joints, ["left_hip", "right_hip"])
    head = _pick_center(joints, ["nose", "left_ear", "right_ear"])
    right_axis_source = None
    up_axis_source = None

    if "left_shoulder" in joints and "right_shoulder" in joints:
        right_axis_source = joints["right_shoulder"] - joints["left_shoulder"]
    elif "left_hip" in joints and "right_hip" in joints:
        right_axis_source = joints["right_hip"] - joints["left_hip"]

    if shoulders is not None and hips is not None:
        up_axis_source = shoulders - hips
    elif head is not None and hips is not None:
        up_axis_source = head - hips

    if right_axis_source is None or up_axis_source is None:
        return Vec3(0.0, 0.0, 1.0)

    _, _, forward = _build_basis(right=right_axis_source, up=up_axis_source)
    planar = Vec3(forward.x, 0.0, forward.z)
    return planar.normalized(Vec3(0.0, 0.0, 1.0))


def _head_anchor(joints: Dict[str, Vec3], trackers: Dict[str, Vec3] | None = None) -> Vec3 | None:
    face = _pick_center(joints, ["nose", "left_eye", "right_eye", "left_ear", "right_ear"])
    if face is not None:
        return face
    if trackers is not None:
        return trackers.get("head") or trackers.get("chest") or trackers.get("waist")
    return None


def _root_anchor(joints: Dict[str, Vec3], trackers: Dict[str, Vec3] | None = None) -> Vec3 | None:
    if trackers is not None:
        root = trackers.get("chest") or trackers.get("neck") or trackers.get("waist")
        if root is not None:
            return root
    return _pick_center(joints, ["left_shoulder", "right_shoulder"]) or _pick_center(joints, ["left_hip", "right_hip"])


def _head_forward(joints: Dict[str, Vec3]) -> Vec3:
    if "nose" in joints:
        reference = _pick_center(joints, ["left_ear", "right_ear"]) or _pick_center(joints, ["left_eye", "right_eye"])
        if reference is not None:
            face_forward = joints["nose"] - reference
            planar = Vec3(face_forward.x, 0.0, face_forward.z)
            if planar.length() > 0.03:
                return planar.normalized(Vec3(0.0, 0.0, 1.0))
    return _torso_forward(joints)


def _derive_controller_pose(
    joints: Dict[str, Vec3],
    side: str,
    fallback_pose: VmtPose | None,
    config: BridgeConfig,
) -> VmtPose | None:
    shoulder = joints.get(f"{side}_shoulder")
    elbow = joints.get(f"{side}_elbow")
    wrist = joints.get(f"{side}_wrist")

    if wrist is None:
        return fallback_pose

    torso_forward = _torso_forward(joints)
    upper_arm = (elbow - shoulder) if elbow is not None and shoulder is not None else None
    forearm = (wrist - elbow) if wrist is not None and elbow is not None else None

    wrist_position = fallback_pose.position if fallback_pose is not None else wrist
    grip_position = wrist_position
    if shoulder is not None:
        arm_vector = wrist_position - shoulder
        if arm_vector.length() > 0.02:
            grip_position = shoulder + (arm_vector * max(config.controller_position_scale, 0.1))

    up_axis = forearm or upper_arm or Vec3(0.0, -1.0, 0.0)
    forward_axis = upper_arm or torso_forward
    basis = _build_basis(up=up_axis, forward=forward_axis, fallback_forward=torso_forward)
    rotation = _quaternion_from_basis(*basis)
    return VmtPose(position=grip_position, rotation=rotation)


def _body_pose(joints: Dict[str, Vec3], tracker_name: str, position: Vec3) -> VmtPose:
    torso_forward = _torso_forward(joints)
    if tracker_name == "head":
        right = None
        head = _head_anchor(joints)
        chest = _pick_center(joints, ["left_shoulder", "right_shoulder"])
        if "left_ear" in joints and "right_ear" in joints:
            right = joints["right_ear"] - joints["left_ear"]
        elif "left_shoulder" in joints and "right_shoulder" in joints:
            right = joints["right_shoulder"] - joints["left_shoulder"]
        up = (head - chest) if head is not None and chest is not None else Vec3(0.0, 1.0, 0.0)
        face_forward = _head_forward(joints)
        basis = _build_basis(
            right=right or Vec3(1.0, 0.0, 0.0),
            forward=face_forward,
            fallback_forward=torso_forward,
        )
        if basis[1].length() <= EPSILON:
            basis = _build_basis(right=right or Vec3(1.0, 0.0, 0.0), up=up, fallback_forward=torso_forward)
    elif tracker_name in {"waist", "chest", "neck"}:
        right = None
        if "left_shoulder" in joints and "right_shoulder" in joints:
            right = joints["right_shoulder"] - joints["left_shoulder"]
        elif "left_hip" in joints and "right_hip" in joints:
            right = joints["right_hip"] - joints["left_hip"]
        chest = _pick_center(joints, ["left_shoulder", "right_shoulder"])
        waist = _pick_center(joints, ["left_hip", "right_hip"])
        head = _pick_center(joints, ["nose", "left_ear", "right_ear"])
        up = (chest - waist) if chest is not None and waist is not None else (head - position if head is not None else Vec3(0.0, 1.0, 0.0))
        basis = _build_basis(right=right or Vec3(1.0, 0.0, 0.0), up=up, fallback_forward=torso_forward)
    elif tracker_name in {"left_hand", "right_hand", "left_elbow", "right_elbow"}:
        side = "left" if tracker_name.startswith("left") else "right"
        shoulder = joints.get(f"{side}_shoulder")
        elbow = joints.get(f"{side}_elbow")
        wrist = joints.get(f"{side}_wrist")
        upper_arm = (elbow - shoulder) if elbow is not None and shoulder is not None else None
        forearm = (wrist - elbow) if wrist is not None and elbow is not None else None

        if tracker_name.endswith("hand"):
            up_axis = forearm or upper_arm or Vec3(0.0, 1.0, 0.0)
            forward_axis = upper_arm or torso_forward
            basis = _build_basis(up=up_axis, forward=forward_axis, fallback_forward=torso_forward)
        else:
            up_axis = upper_arm or forearm or Vec3(0.0, 1.0, 0.0)
            forward_axis = forearm or torso_forward
            basis = _build_basis(up=up_axis, forward=forward_axis, fallback_forward=torso_forward)
    elif tracker_name in {"left_foot", "right_foot", "left_knee", "right_knee"}:
        side = "left" if tracker_name.startswith("left") else "right"
        ankle = joints.get(f"{side}_ankle")
        toe = joints.get(f"{side}_foot_index")
        heel = joints.get(f"{side}_heel")
        leg_up = None
        if tracker_name.endswith("foot"):
            if toe is not None and heel is not None:
                forward = toe - heel
            elif toe is not None and ankle is not None:
                forward = toe - ankle
            else:
                forward = torso_forward
            basis = _build_basis(up=Vec3(0.0, 1.0, 0.0), forward=forward, fallback_forward=torso_forward)
        else:
            knee = joints.get(f"{side}_knee")
            hip = joints.get(f"{side}_hip")
            leg_up = (knee - hip) if knee is not None and hip is not None else Vec3(0.0, 1.0, 0.0)
            basis = _build_basis(up=leg_up, forward=torso_forward)
    else:
        basis = _build_basis(forward=torso_forward, up=Vec3(0.0, 1.0, 0.0))

    rotation = _quaternion_from_basis(*basis)
    return VmtPose(position=position, rotation=rotation)


def _extract_points(payload: Dict[str, object], key: str, config: BridgeConfig) -> Dict[str, Vec3]:
    raw_points = payload.get(key, {})
    if not isinstance(raw_points, dict):
        return {}

    parsed: Dict[str, Vec3] = {}
    for name, vector_payload in raw_points.items():
        if not isinstance(name, str) or not isinstance(vector_payload, dict):
            continue
        try:
            parsed[name] = _scale_axes(_parse_vec3(vector_payload), config)
        except (KeyError, TypeError, ValueError):
            continue
    return parsed


def _calibrate(
    state: CalibrationState,
    joints: Dict[str, Vec3],
    trackers: Dict[str, Vec3],
    config: BridgeConfig,
) -> None:
    waist = trackers.get("waist") or _pick_center(joints, ["left_hip", "right_hip"])
    if waist is None:
        return

    forward = _torso_forward(joints)
    state.origin = waist
    state.yaw_correction = math.atan2(forward.x, forward.z)

    feet = [
        point.y - waist.y
        for point in (
            trackers.get("left_foot"),
            trackers.get("right_foot"),
            joints.get("left_ankle"),
            joints.get("right_ankle"),
        )
        if point is not None
    ]
    if feet:
        state.y_offset = max(config.foot_floor_offset, -min(feet) + config.foot_floor_offset)
    else:
        state.y_offset = config.waist_height

    head_anchor = trackers.get("head") or trackers.get("chest") or trackers.get("waist")
    if head_anchor is not None:
        state.hmd_origin = head_anchor
        state.hmd_reference = state.transform(head_anchor, config)
    root_anchor = _root_anchor(joints, trackers)
    if head_anchor is not None and root_anchor is not None:
        state.hmd_root_origin = root_anchor
        state.hmd_root_offset = state.transform_delta(head_anchor - root_anchor, config)
    state.ready = True


def _derive_vmt_poses(joints: Dict[str, Vec3], trackers: Dict[str, Vec3]) -> Dict[str, VmtPose]:
    poses: Dict[str, VmtPose] = {}
    for tracker_name, tracker_position in trackers.items():
        poses[tracker_name] = _body_pose(joints, tracker_name, tracker_position)
    return poses


def _derive_head_hmd_pose(
    calibration: CalibrationState,
    config: BridgeConfig,
    raw_joints: Dict[str, Vec3],
    raw_trackers: Dict[str, Vec3],
    transformed_joints: Dict[str, Vec3],
    transformed_trackers: Dict[str, Vec3],
    fallback_pose: VmtPose | None,
) -> VmtPose | None:
    head_anchor = _head_anchor(raw_joints, raw_trackers)
    if head_anchor is None:
        return fallback_pose
    root_anchor = _root_anchor(raw_joints, raw_trackers)
    transformed_head = _head_anchor(transformed_joints, transformed_trackers)

    if calibration.hmd_origin is None or calibration.hmd_reference is None:
        calibration.hmd_origin = head_anchor
        calibration.hmd_reference = calibration.transform(head_anchor, config)
    if root_anchor is not None and calibration.hmd_root_origin is None:
        calibration.hmd_root_origin = root_anchor
    if root_anchor is not None and calibration.hmd_root_offset is None:
        calibration.hmd_root_offset = calibration.transform_delta(head_anchor - root_anchor, config)

    position = None
    if transformed_head is not None:
        # Keep the HMD in the same transformed space as the body trackers first.
        position = transformed_head
    elif fallback_pose is not None:
        position = fallback_pose.position

    transformed_root = _root_anchor(transformed_joints, transformed_trackers)
    if position is None and transformed_root is not None and calibration.hmd_root_offset is not None:
        position = transformed_root + calibration.hmd_root_offset
        if root_anchor is not None:
            live_root_to_head = calibration.transform_delta(head_anchor - root_anchor, config)
            local_delta = live_root_to_head - calibration.hmd_root_offset
            position = position + Vec3(
                local_delta.x * HMD_LOCAL_XZ_BLEND,
                local_delta.y * HMD_LOCAL_Y_BLEND,
                local_delta.z * HMD_LOCAL_XZ_BLEND,
            )

    if position is None:
        delta = calibration.transform_delta(head_anchor - calibration.hmd_origin, config)
        position = calibration.hmd_reference + delta

    position = Vec3(position.x, position.y + config.hmd_y_offset, position.z)
    chest_position = transformed_trackers.get("chest") or transformed_trackers.get("waist")
    if chest_position is not None and position.y < (chest_position.y + 0.18):
        position = Vec3(position.x, chest_position.y + 0.18, position.z)

    if fallback_pose is not None:
        return VmtPose(position=position, rotation=fallback_pose.rotation)

    synthetic_joints = dict(transformed_joints)
    synthetic_joints["nose"] = position
    synthetic_joints["left_ear"] = position
    synthetic_joints["right_ear"] = position
    return _body_pose(synthetic_joints, "head", position)


class VmtOscClient:
    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_pose(self, binding: DeviceBinding, pose: VmtPose) -> None:
        self._socket.sendto(
            _encode_osc_message(
                "/VMT/Room/Unity",
                [
                    binding.index,
                    binding.enable_mode,
                    0.0,
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.rotation.x,
                    pose.rotation.y,
                    pose.rotation.z,
                    pose.rotation.w,
                ],
            ),
            (self._host, self._port),
        )

    def disable(self, binding: DeviceBinding) -> None:
        self._socket.sendto(
            _encode_osc_message(
                "/VMT/Room/Unity",
                [binding.index, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            ),
            (self._host, self._port),
        )

    def close(self) -> None:
        self._socket.close()

    def send_button(self, index: int, button_index: int, pressed: bool) -> None:
        self._socket.sendto(
            _encode_osc_message("/VMT/Input/Button", [index, button_index, 0.0, 1 if pressed else 0]),
            (self._host, self._port),
        )
        self._socket.sendto(
            _encode_osc_message("/VMT/Input/Button/Touch", [index, button_index, 0.0, 1 if pressed else 0]),
            (self._host, self._port),
        )

    def send_trigger(self, index: int, trigger_index: int, value: float) -> None:
        pressed = value > 0.75
        touched = value > 0.10
        self._socket.sendto(
            _encode_osc_message("/VMT/Input/Trigger", [index, trigger_index, 0.0, _clamp(value, 0.0, 1.0)]),
            (self._host, self._port),
        )
        self._socket.sendto(
            _encode_osc_message("/VMT/Input/Trigger/Touch", [index, trigger_index, 0.0, 1 if touched else 0]),
            (self._host, self._port),
        )
        self._socket.sendto(
            _encode_osc_message("/VMT/Input/Trigger/Click", [index, trigger_index, 0.0, 1 if pressed else 0]),
            (self._host, self._port),
        )

    def send_joystick(self, index: int, joystick_index: int, x_value: float, y_value: float, clicked: bool) -> None:
        touched = abs(x_value) > 0.05 or abs(y_value) > 0.05
        self._socket.sendto(
            _encode_osc_message(
                "/VMT/Input/Joystick",
                [index, joystick_index, 0.0, _clamp(x_value, -1.0, 1.0), _clamp(y_value, -1.0, 1.0)],
            ),
            (self._host, self._port),
        )
        self._socket.sendto(
            _encode_osc_message("/VMT/Input/Joystick/Touch", [index, joystick_index, 0.0, 1 if touched else 0]),
            (self._host, self._port),
        )
        self._socket.sendto(
            _encode_osc_message("/VMT/Input/Joystick/Click", [index, joystick_index, 0.0, 1 if clicked else 0]),
            (self._host, self._port),
        )

    def send_controller_state(self, index: int, state: ControllerGestureState) -> None:
        self.send_trigger(index, 1, state.grip)


class HeadlessHmdUdpClient:
    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_pose(self, pose: VmtPose, connected: bool = True) -> None:
        payload = (
            f"{pose.position.x:.6f} {pose.position.y:.6f} {pose.position.z:.6f} "
            f"{pose.rotation.x:.6f} {pose.rotation.y:.6f} {pose.rotation.z:.6f} {pose.rotation.w:.6f} "
            f"{1 if connected else 0}"
        ).encode("utf-8")
        self._socket.sendto(payload, (self._host, self._port))

    def close(self) -> None:
        self._socket.close()


class ControllerGestureState:
    def __init__(self, trigger: float, grip: float, joystick_x: float, joystick_y: float, a_pressed: bool, b_pressed: bool):
        self.trigger = trigger
        self.grip = grip
        self.joystick_x = joystick_x
        self.joystick_y = joystick_y
        self.a_pressed = a_pressed
        self.b_pressed = b_pressed


def _neutral_controller_state() -> ControllerGestureState:
    return ControllerGestureState(0.0, 0.0, 0.0, 0.0, False, False)


def _is_key_pressed(virtual_key: int) -> bool:
    if not sys.platform.startswith("win"):
        return False
    return bool(ctypes.windll.user32.GetAsyncKeyState(virtual_key) & 0x8000)


def _keyboard_joystick_state() -> tuple[float, float]:
    left_pressed = _is_key_pressed(0x41)  # A
    right_pressed = _is_key_pressed(0x44)  # D
    forward_pressed = _is_key_pressed(0x57)  # W
    back_pressed = _is_key_pressed(0x53)  # S

    x_value = float(right_pressed) - float(left_pressed)
    y_value = float(forward_pressed) - float(back_pressed)
    return _clamp(x_value, -1.0, 1.0), _clamp(y_value, -1.0, 1.0)


def _gesture_state_for_side(joints: Dict[str, Vec3], side: str) -> ControllerGestureState | None:
    wrist = joints.get(f"{side}_wrist")
    elbow = joints.get(f"{side}_elbow")
    shoulder = joints.get(f"{side}_shoulder")
    thumb = joints.get(f"{side}_thumb")
    index = joints.get(f"{side}_index")
    pinky = joints.get(f"{side}_pinky")

    if wrist is None or shoulder is None:
        return None

    forearm_length = _distance(wrist, elbow) if elbow is not None else 0.25
    forearm_length = max(forearm_length, 0.18)

    if index is not None and pinky is not None:
        span_ratio = _distance(index, pinky) / forearm_length
        grip = 1.0 - _clamp((span_ratio - 0.08) / 0.30, 0.0, 1.0)
    else:
        grip = 0.0

    return ControllerGestureState(0.0, grip, 0.0, 0.0, False, False)


class VmtBridgeRuntime:
    def __init__(self, config: BridgeConfig) -> None:
        self.config = config
        self.calibration = CalibrationState()
        self.client = VmtOscClient(config.vmt_host, config.vmt_port)
        self.hmd_client = HeadlessHmdUdpClient(config.hmd_host, config.hmd_port) if config.enable_hmd else None
        self._last_frame_time = 0.0
        self._last_print_time = 0.0
        self._enabled_names: set[str] = set()
        self._enabled_controllers: set[str] = set()

    def _disable_controller(self, binding: ControllerBinding) -> None:
        self.client.send_controller_state(binding.index, _neutral_controller_state())
        self.client.disable(DeviceBinding(binding.index, binding.enable_mode))

    def process_packet(self, payload: Dict[str, object]) -> int:
        joints = _extract_points(payload, "joints", self.config)
        trackers = _extract_points(payload, "trackers", self.config)
        if not joints or not trackers:
            return 0

        if not self.calibration.ready:
            _calibrate(self.calibration, joints, trackers, self.config)
            if self.calibration.ready:
                print(
                    "Calibrated VMT bridge: "
                    f"yaw={math.degrees(self.calibration.yaw_correction):.1f}deg "
                    f"floor_offset={self.calibration.y_offset:.2f}m"
                )

        if not self.calibration.ready:
            return 0

        transformed_joints = {
            name: self.calibration.transform(vector, self.config)
            for name, vector in joints.items()
        }
        transformed_trackers = {
            name: self.calibration.transform(vector, self.config)
            for name, vector in trackers.items()
        }
        poses = _derive_vmt_poses(transformed_joints, transformed_trackers)

        sent = 0
        active_names: set[str] = set()
        for name, binding in self.config.bindings.items():
            pose = poses.get(name)
            if pose is None:
                continue
            self.client.send_pose(binding, pose)
            sent += 1
            active_names.add(name)

        for stale_name in self._enabled_names - active_names:
            binding = self.config.bindings.get(stale_name)
            if binding is not None:
                self.client.disable(binding)
        self._enabled_names = active_names

        active_controllers: set[str] = set()
        if self.config.enable_controllers:
            for controller_name, binding in self.config.controller_bindings.items():
                hand_pose = _derive_controller_pose(
                    transformed_joints,
                    binding.side,
                    poses.get("left_hand" if binding.side == "left" else "right_hand"),
                    self.config,
                )
                gesture_state = _gesture_state_for_side(transformed_joints, binding.side)
                if hand_pose is None or gesture_state is None:
                    continue
                self.client.send_pose(DeviceBinding(binding.index, binding.enable_mode), hand_pose)
                self.client.send_controller_state(binding.index, gesture_state)
                active_controllers.add(controller_name)

        for stale_name in self._enabled_controllers - active_controllers:
            binding = self.config.controller_bindings.get(stale_name)
            if binding is not None:
                self._disable_controller(binding)
        self._enabled_controllers = active_controllers

        if self.hmd_client is not None:
            head_pose = _derive_head_hmd_pose(
                calibration=self.calibration,
                config=self.config,
                raw_joints=joints,
                raw_trackers=trackers,
                transformed_joints=transformed_joints,
                transformed_trackers=transformed_trackers,
                fallback_pose=poses.get("head") or poses.get("chest") or poses.get("waist"),
            )
            if head_pose is not None:
                self.hmd_client.send_pose(head_pose, connected=True)

        self._last_frame_time = time.time()

        now = time.time()
        if (now - self._last_print_time) >= self.config.print_every:
            self._last_print_time = now
            mode = str(payload.get("mode", "unknown"))
            print(
                f"Forwarded {sent} tracker poses to VMT | mode={mode} | "
                f"active={sorted(active_names)} | controllers={sorted(active_controllers)}"
            )
        return sent

    def tick(self) -> None:
        if not self._enabled_names:
            if self.hmd_client is not None and (time.time() - self._last_frame_time) >= self.config.packet_timeout:
                neutral = VmtPose(position=Vec3(0.0, 1.65, 0.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0))
                self.hmd_client.send_pose(neutral, connected=False)
            if self._enabled_controllers and (time.time() - self._last_frame_time) >= self.config.packet_timeout:
                for stale_name in list(self._enabled_controllers):
                    binding = self.config.controller_bindings.get(stale_name)
                    if binding is not None:
                        self._disable_controller(binding)
                self._enabled_controllers.clear()
            return
        if (time.time() - self._last_frame_time) < self.config.packet_timeout:
            return
        for name in list(self._enabled_names):
            binding = self.config.bindings.get(name)
            if binding is not None:
                self.client.disable(binding)
        self._enabled_names.clear()
        for stale_name in list(self._enabled_controllers):
            binding = self.config.controller_bindings.get(stale_name)
            if binding is not None:
                self._disable_controller(binding)
        self._enabled_controllers.clear()
        if self.hmd_client is not None:
            neutral = VmtPose(position=Vec3(0.0, 1.65, 0.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0))
            self.hmd_client.send_pose(neutral, connected=False)

    def close(self) -> None:
        for name in list(self._enabled_names):
            binding = self.config.bindings.get(name)
            if binding is not None:
                self.client.disable(binding)
        self._enabled_names.clear()
        for stale_name in list(self._enabled_controllers):
            binding = self.config.controller_bindings.get(stale_name)
            if binding is not None:
                self._disable_controller(binding)
        self._enabled_controllers.clear()
        self.client.close()
        if self.hmd_client is not None:
            neutral = VmtPose(position=Vec3(0.0, 1.65, 0.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0))
            self.hmd_client.send_pose(neutral, connected=False)
            self.hmd_client.close()


def _parse_args(argv: List[str]) -> BridgeConfig:
    parser = argparse.ArgumentParser(
        description="Receive JSON/UDP tracking frames and forward them to Virtual Motion Tracker (VMT)."
    )
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=7000)
    parser.add_argument("--vmt-host", default="127.0.0.1")
    parser.add_argument("--vmt-port", type=int, default=39570)
    parser.add_argument("--hmd-host", default="127.0.0.1")
    parser.add_argument("--hmd-port", type=int, default=39575)
    parser.add_argument("--disable-hmd", action="store_true", help="Forward trackers only and skip the virtual HMD.")
    parser.add_argument("--disable-controllers", action="store_true", help="Disable hand-based virtual controller output.")
    parser.add_argument("--scale", type=float, default=1.0, help="Global position scale after calibration.")
    parser.add_argument("--x-axis-scale", type=float, default=1.0)
    parser.add_argument("--y-axis-scale", type=float, default=1.0)
    parser.add_argument("--z-axis-scale", type=float, default=-1.0)
    parser.add_argument("--hmd-y-offset", type=float, default=0.0)
    parser.add_argument("--controller-position-scale", type=float, default=2.2)
    parser.add_argument("--waist-height", type=float, default=0.95, help="Fallback waist height in meters.")
    parser.add_argument("--foot-floor-offset", type=float, default=0.02, help="Lift feet slightly above floor.")
    parser.add_argument("--packet-timeout", type=float, default=1.5)
    parser.add_argument("--print-every", type=float, default=1.0)
    args = parser.parse_args(argv)
    return BridgeConfig(
        listen_host=args.listen_host,
        listen_port=args.listen_port,
        vmt_host=args.vmt_host,
        vmt_port=args.vmt_port,
        hmd_host=args.hmd_host,
        hmd_port=args.hmd_port,
        enable_hmd=not args.disable_hmd,
        enable_controllers=not args.disable_controllers,
        scale=args.scale,
        x_axis_scale=args.x_axis_scale,
        y_axis_scale=args.y_axis_scale,
        z_axis_scale=args.z_axis_scale,
        hmd_y_offset=args.hmd_y_offset,
        controller_position_scale=args.controller_position_scale,
        waist_height=args.waist_height,
        foot_floor_offset=args.foot_floor_offset,
        packet_timeout=args.packet_timeout,
        print_every=args.print_every,
    )


def main(argv: List[str] | None = None) -> int:
    config = _parse_args(argv or sys.argv[1:])
    runtime = VmtBridgeRuntime(config)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((config.listen_host, config.listen_port))
    sock.settimeout(0.5)

    print(
        f"Listening for tracker packets on udp://{config.listen_host}:{config.listen_port} "
        f"and forwarding to VMT on udp://{config.vmt_host}:{config.vmt_port}"
    )
    if config.enable_hmd:
        print(f"Forwarding head pose to virtual HMD on udp://{config.hmd_host}:{config.hmd_port}")
    if config.enable_controllers:
        print("Hand gestures enabled as VMT compatible controllers at indexes 20(left) and 21(right)")

    try:
        while True:
            try:
                payload, _ = sock.recvfrom(65535)
            except socket.timeout:
                runtime.tick()
                continue

            try:
                packet = json.loads(payload.decode("utf-8"))
            except json.JSONDecodeError:
                continue

            if isinstance(packet, dict):
                runtime.process_packet(packet)
    except KeyboardInterrupt:
        print("Stopping VMT bridge...")
    finally:
        runtime.close()
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
