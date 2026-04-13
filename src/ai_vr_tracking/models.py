from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List


class TrackingMode(str, Enum):
    FULL_BODY = "full_body"
    UPPER_BODY = "upper_body"


@dataclass
class CameraConfig:
    camera_index: int
    enabled: bool = True
    weight: float = 1.0
    mirror: bool = True

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "CameraConfig":
        return cls(
            camera_index=int(payload.get("camera_index", 0)),
            enabled=bool(payload.get("enabled", True)),
            weight=float(payload.get("weight", 1.0)),
            mirror=bool(payload.get("mirror", True)),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "camera_index": self.camera_index,
            "enabled": self.enabled,
            "weight": self.weight,
            "mirror": self.mirror,
        }


@dataclass
class TrackingConfig:
    mode: TrackingMode = TrackingMode.FULL_BODY
    min_visibility: float = 0.55
    smoothing: float = 0.45
    output_fps: float = 20.0
    model_complexity: int = 1
    max_cameras_to_scan: int = 5
    pose_model_path: str = "models/pose_landmarker_full.task"

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "TrackingConfig":
        mode = payload.get("mode", TrackingMode.FULL_BODY.value)
        return cls(
            mode=TrackingMode(mode),
            min_visibility=float(payload.get("min_visibility", 0.55)),
            smoothing=float(payload.get("smoothing", 0.45)),
            output_fps=float(payload.get("output_fps", 20.0)),
            model_complexity=int(payload.get("model_complexity", 1)),
            max_cameras_to_scan=int(payload.get("max_cameras_to_scan", 5)),
            pose_model_path=str(payload.get("pose_model_path", "models/pose_landmarker_full.task")),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "mode": self.mode.value,
            "min_visibility": self.min_visibility,
            "smoothing": self.smoothing,
            "output_fps": self.output_fps,
            "model_complexity": self.model_complexity,
            "max_cameras_to_scan": self.max_cameras_to_scan,
            "pose_model_path": self.pose_model_path,
        }


@dataclass
class OutputConfig:
    enabled: bool = True
    protocol: str = "osc"
    host: str = "127.0.0.1"
    port: int = 9000
    emit_joints: bool = True

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "OutputConfig":
        return cls(
            enabled=bool(payload.get("enabled", True)),
            protocol=str(payload.get("protocol", "osc")),
            host=str(payload.get("host", "127.0.0.1")),
            port=int(payload.get("port", 9000)),
            emit_joints=bool(payload.get("emit_joints", True)),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "enabled": self.enabled,
            "protocol": self.protocol,
            "host": self.host,
            "port": self.port,
            "emit_joints": self.emit_joints,
        }


@dataclass
class AppConfig:
    tracking: TrackingConfig = field(default_factory=TrackingConfig)
    output: OutputConfig = field(default_factory=OutputConfig)
    cameras: List[CameraConfig] = field(
        default_factory=lambda: [CameraConfig(camera_index=0), CameraConfig(camera_index=1, enabled=False)]
    )

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "AppConfig":
        cameras = payload.get("cameras", [])
        return cls(
            tracking=TrackingConfig.from_dict(payload.get("tracking", {})),
            output=OutputConfig.from_dict(payload.get("output", {})),
            cameras=[CameraConfig.from_dict(item) for item in cameras] or [CameraConfig(camera_index=0)],
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "tracking": self.tracking.to_dict(),
            "output": self.output.to_dict(),
            "cameras": [camera.to_dict() for camera in self.cameras],
        }


@dataclass
class JointSample:
    name: str
    x: float
    y: float
    z: float
    visibility: float

    def to_dict(self) -> Dict[str, float]:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "visibility": self.visibility,
        }


@dataclass
class TrackerPose:
    name: str
    x: float
    y: float
    z: float
    confidence: float

    def to_dict(self) -> Dict[str, float]:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "confidence": self.confidence,
        }


@dataclass
class PoseObservation:
    camera_index: int
    camera_weight: float
    joints: Dict[str, JointSample]
    score: float


@dataclass
class FusedFrame:
    timestamp: float
    mode: TrackingMode
    joints: Dict[str, JointSample]
    trackers: Dict[str, TrackerPose]
    active_cameras: List[int]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "mode": self.mode.value,
            "active_cameras": self.active_cameras,
            "joints": {name: joint.to_dict() for name, joint in self.joints.items()},
            "trackers": {name: tracker.to_dict() for name, tracker in self.trackers.items()},
        }


@dataclass
class TrackingUpdate:
    fused_frame: FusedFrame | None = None
    camera_previews: Dict[int, Any] = field(default_factory=dict)
    fused_preview: Any | None = None
    fps: float = 0.0
    note: str = ""
    active_camera_count: int = 0

