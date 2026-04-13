from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, List, Tuple

from .constants import LANDMARK_NAMES, POSE_CONNECTIONS, TRACKER_COLORS, UPPER_BODY_JOINTS
from .models import (
    AppConfig,
    CameraConfig,
    FusedFrame,
    JointSample,
    PoseObservation,
    TrackerPose,
    TrackingMode,
    TrackingUpdate,
)
from .output import TrackingOutputRouter


def detect_missing_runtime_dependencies() -> List[str]:
    missing = []
    for module_name, package_name in (
        ("cv2", "opencv-python"),
        ("mediapipe", "mediapipe"),
        ("numpy", "numpy"),
        ("PIL", "Pillow"),
    ):
        try:
            __import__(module_name)
        except ImportError:
            missing.append(package_name)
    return missing


def list_available_cameras(max_index: int = 5) -> List[int]:
    try:
        import cv2
    except ImportError:
        return []

    cameras: List[int] = []
    for index in range(max_index + 1):
        capture = cv2.VideoCapture(index)
        try:
            if not capture.isOpened():
                continue
            ok, _ = capture.read()
            if ok:
                cameras.append(index)
        finally:
            capture.release()
    return cameras


def _joint_names_for_mode(mode: TrackingMode) -> Iterable[str]:
    if mode == TrackingMode.UPPER_BODY:
        return [name for name in LANDMARK_NAMES if name in UPPER_BODY_JOINTS]
    return LANDMARK_NAMES


def _weighted_average(samples: List[Tuple[JointSample, float]]) -> Tuple[Tuple[float, float, float], float]:
    import numpy as np

    original_points = np.array([[sample.x, sample.y, sample.z] for sample, _ in samples], dtype=float)
    original_weights = np.array([weight for _, weight in samples], dtype=float)
    original_visibilities = np.array([sample.visibility for sample, _ in samples], dtype=float)
    points = original_points
    weights = original_weights
    visibilities = original_visibilities

    if len(points) > 2:
        center = np.median(points, axis=0)
        distances = np.linalg.norm(points - center, axis=1)
        threshold = max(0.15, float(np.median(distances) * 2.5))
        mask = distances <= threshold
        if mask.any():
            points = points[mask]
            weights = weights[mask]
            visibilities = visibilities[mask]
        else:
            points = original_points
            weights = original_weights
            visibilities = original_visibilities

    if weights.sum() <= 0:
        weights = np.ones(len(points), dtype=float)

    averaged = np.average(points, axis=0, weights=weights)
    confidence = float(np.average(visibilities, weights=weights))
    return (float(averaged[0]), float(averaged[1]), float(averaged[2])), confidence


def _average_named_points(joints: Dict[str, JointSample], names: Iterable[str]) -> TrackerPose | None:
    selected = [joints[name] for name in names if name in joints]
    if not selected:
        return None
    count = float(len(selected))
    return TrackerPose(
        name="",
        x=sum(item.x for item in selected) / count,
        y=sum(item.y for item in selected) / count,
        z=sum(item.z for item in selected) / count,
        confidence=sum(item.visibility for item in selected) / count,
    )


def _build_trackers(joints: Dict[str, JointSample], mode: TrackingMode) -> Dict[str, TrackerPose]:
    trackers: Dict[str, TrackerPose] = {}

    head = _average_named_points(joints, ["nose", "left_eye", "right_eye", "left_ear", "right_ear"])
    shoulders = _average_named_points(joints, ["left_shoulder", "right_shoulder"])
    hips = _average_named_points(joints, ["left_hip", "right_hip"])
    left_hand = _average_named_points(joints, ["left_wrist", "left_index", "left_thumb", "left_pinky"])
    right_hand = _average_named_points(joints, ["right_wrist", "right_index", "right_thumb", "right_pinky"])
    left_elbow = _average_named_points(joints, ["left_elbow"])
    right_elbow = _average_named_points(joints, ["right_elbow"])
    left_knee = _average_named_points(joints, ["left_knee"])
    right_knee = _average_named_points(joints, ["right_knee"])
    left_foot = _average_named_points(joints, ["left_ankle", "left_heel", "left_foot_index"])
    right_foot = _average_named_points(joints, ["right_ankle", "right_heel", "right_foot_index"])

    if head is not None:
        head.name = "head"
        trackers["head"] = head

    if shoulders is not None:
        shoulders.name = "neck"
        trackers["neck"] = shoulders

    if shoulders is not None and hips is not None:
        chest = TrackerPose(
            name="chest",
            x=(shoulders.x * 0.7) + (hips.x * 0.3),
            y=(shoulders.y * 0.7) + (hips.y * 0.3),
            z=(shoulders.z * 0.7) + (hips.z * 0.3),
            confidence=(shoulders.confidence + hips.confidence) / 2,
        )
        trackers["chest"] = chest

    if hips is not None:
        hips.name = "waist"
        trackers["waist"] = hips

    for name, tracker in (
        ("left_hand", left_hand),
        ("right_hand", right_hand),
        ("left_elbow", left_elbow),
        ("right_elbow", right_elbow),
    ):
        if tracker is not None:
            tracker.name = name
            trackers[name] = tracker

    if mode == TrackingMode.FULL_BODY:
        for name, tracker in (
            ("left_knee", left_knee),
            ("right_knee", right_knee),
            ("left_foot", left_foot),
            ("right_foot", right_foot),
        ):
            if tracker is not None:
                tracker.name = name
                trackers[name] = tracker

    return trackers


def _render_fused_preview(frame: FusedFrame | None, note: str) -> Any | None:
    try:
        import cv2
        import numpy as np
    except ImportError:
        return None

    canvas = np.zeros((540, 720, 3), dtype=np.uint8)
    canvas[:] = (17, 20, 29)
    cv2.putText(canvas, "Fused Skeleton", (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (250, 250, 250), 2)
    cv2.putText(canvas, note[:80], (18, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (190, 196, 214), 1)

    if frame is None or not frame.joints:
        cv2.putText(canvas, "No body detected yet", (18, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (140, 146, 172), 2)
        return canvas

    hips = _average_named_points(frame.joints, ["left_hip", "right_hip"])
    origin_x = hips.x if hips is not None else 0.0
    origin_y = hips.y if hips is not None else 0.0
    scale = 260.0

    def project(sample: JointSample) -> Tuple[int, int]:
        px = int(360 + (sample.x - origin_x) * scale)
        py = int(440 - (sample.y - origin_y) * scale)
        return px, py

    active_joints = set(frame.joints)
    for start, end in POSE_CONNECTIONS:
        if start not in active_joints or end not in active_joints:
            continue
        start_point = project(frame.joints[start])
        end_point = project(frame.joints[end])
        cv2.line(canvas, start_point, end_point, (93, 110, 140), 2)

    for joint in frame.joints.values():
        point = project(joint)
        radius = max(2, int(3 + (joint.visibility * 2)))
        cv2.circle(canvas, point, radius, (221, 227, 236), -1)

    for name, tracker in frame.trackers.items():
        fake_joint = JointSample(name=name, x=tracker.x, y=tracker.y, z=tracker.z, visibility=tracker.confidence)
        point = project(fake_joint)
        color = TRACKER_COLORS.get(name, (120, 200, 255))
        cv2.circle(canvas, point, 8, color, -1)
        cv2.putText(canvas, name, (point[0] + 10, point[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    cv2.putText(
        canvas,
        f"Mode: {frame.mode.value} | Cameras: {','.join(str(item) for item in frame.active_cameras)}",
        (18, 518),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (220, 223, 229),
        1,
    )
    return canvas


@dataclass
class _CameraRuntime:
    config: CameraConfig
    capture: Any
    pose: Any
    draw_utils: Any
    draw_styles: Any
    pose_connections: Any

    def read(self) -> Tuple[PoseObservation | None, Any | None]:
        import cv2

        ok, frame = self.capture.read()
        if not ok:
            return None, None

        if self.config.mirror:
            frame = cv2.flip(frame, 1)

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(rgb_frame)
        preview = frame.copy()

        if results.pose_landmarks is not None:
            self.draw_utils.draw_landmarks(
                preview,
                results.pose_landmarks,
                self.pose_connections,
                landmark_drawing_spec=self.draw_styles.get_default_pose_landmarks_style(),
            )

        observation = _results_to_observation(results, self.config)
        label = f"Cam {self.config.camera_index} {'OK' if observation else 'No pose'}"
        color = (80, 220, 140) if observation else (120, 130, 150)
        cv2.putText(preview, label, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        return observation, preview

    def close(self) -> None:
        self.capture.release()
        self.pose.close()


def _results_to_observation(results: Any, config: CameraConfig) -> PoseObservation | None:
    if results.pose_landmarks is None:
        return None

    landmark_source = results.pose_world_landmarks or results.pose_landmarks
    joints: Dict[str, JointSample] = {}
    visibilities = []
    for name, landmark in zip(LANDMARK_NAMES, landmark_source.landmark):
        visibility = float(getattr(landmark, "visibility", 1.0))
        visibilities.append(visibility)
        joints[name] = JointSample(
            name=name,
            x=float(landmark.x),
            y=float(landmark.y),
            z=float(landmark.z),
            visibility=visibility,
        )

    score = sum(visibilities) / max(len(visibilities), 1)
    return PoseObservation(
        camera_index=config.camera_index,
        camera_weight=config.weight,
        joints=joints,
        score=score,
    )


def _fuse_observations(
    observations: List[PoseObservation],
    mode: TrackingMode,
    min_visibility: float,
    smoothing: float,
    previous_joints: Dict[str, JointSample],
) -> Dict[str, JointSample]:
    fused: Dict[str, JointSample] = {}

    for joint_name in _joint_names_for_mode(mode):
        collected: List[Tuple[JointSample, float]] = []
        for observation in observations:
            sample = observation.joints.get(joint_name)
            if sample is None or sample.visibility < min_visibility:
                continue
            collected.append((sample, observation.camera_weight * max(sample.visibility, 0.01)))

        if not collected:
            continue

        (x_value, y_value, z_value), confidence = _weighted_average(collected)
        if joint_name in previous_joints:
            previous = previous_joints[joint_name]
            blend = max(0.0, min(0.95, smoothing))
            x_value = (previous.x * blend) + (x_value * (1.0 - blend))
            y_value = (previous.y * blend) + (y_value * (1.0 - blend))
            z_value = (previous.z * blend) + (z_value * (1.0 - blend))
            confidence = (previous.visibility * blend) + (confidence * (1.0 - blend))

        fused[joint_name] = JointSample(
            name=joint_name,
            x=x_value,
            y=y_value,
            z=z_value,
            visibility=confidence,
        )

    return fused


class TrackingEngine:
    def __init__(self, config: AppConfig, output_router: TrackingOutputRouter) -> None:
        self._config = config
        self._output_router = output_router
        self._callback: Callable[[TrackingUpdate], None] | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    def start(self, callback: Callable[[TrackingUpdate], None]) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._callback = callback
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None

    def _emit_update(self, update: TrackingUpdate) -> None:
        if self._callback is not None:
            self._callback(update)

    def _build_camera_runtimes(self) -> List[_CameraRuntime]:
        import cv2
        import mediapipe as mp

        active_cameras = [camera for camera in self._config.cameras if camera.enabled]
        runtimes: List[_CameraRuntime] = []

        for camera in active_cameras:
            capture = cv2.VideoCapture(camera.camera_index)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
            if not capture.isOpened():
                capture.release()
                continue

            pose = mp.solutions.pose.Pose(
                static_image_mode=False,
                model_complexity=self._config.tracking.model_complexity,
                smooth_landmarks=True,
                enable_segmentation=False,
                min_detection_confidence=max(0.3, self._config.tracking.min_visibility - 0.1),
                min_tracking_confidence=max(0.3, self._config.tracking.min_visibility - 0.1),
            )
            runtimes.append(
                _CameraRuntime(
                    config=camera,
                    capture=capture,
                    pose=pose,
                    draw_utils=mp.solutions.drawing_utils,
                    draw_styles=mp.solutions.drawing_styles,
                    pose_connections=mp.solutions.pose.POSE_CONNECTIONS,
                )
            )
        return runtimes

    def _run(self) -> None:
        missing = detect_missing_runtime_dependencies()
        if missing:
            self._emit_update(TrackingUpdate(note=f"Missing dependencies: {', '.join(missing)}"))
            return

        runtimes = self._build_camera_runtimes()
        if not runtimes:
            self._emit_update(TrackingUpdate(note="No available webcams could be opened."))
            return

        previous_joints: Dict[str, JointSample] = {}
        last_output_time = 0.0
        last_tick = time.perf_counter()

        try:
            while not self._stop_event.is_set():
                previews: Dict[int, Any] = {}
                observations: List[PoseObservation] = []

                for runtime in runtimes:
                    observation, preview = runtime.read()
                    if preview is not None:
                        previews[runtime.config.camera_index] = preview
                    if observation is not None:
                        observations.append(observation)

                fused_frame = None
                note = "Tracking active"
                if observations:
                    fused_joints = _fuse_observations(
                        observations=observations,
                        mode=self._config.tracking.mode,
                        min_visibility=self._config.tracking.min_visibility,
                        smoothing=self._config.tracking.smoothing,
                        previous_joints=previous_joints,
                    )
                    trackers = _build_trackers(fused_joints, self._config.tracking.mode)
                    fused_frame = FusedFrame(
                        timestamp=time.time(),
                        mode=self._config.tracking.mode,
                        joints=fused_joints,
                        trackers=trackers,
                        active_cameras=[observation.camera_index for observation in observations],
                    )
                    previous_joints = fused_joints

                    frame_interval = 1.0 / max(self._config.tracking.output_fps, 1.0)
                    send_failed = False
                    if (time.time() - last_output_time) >= frame_interval:
                        try:
                            self._output_router.send(fused_frame)
                            last_output_time = time.time()
                        except OSError as error:
                            note = f"Output send failed: {error}"
                            send_failed = True
                    if not send_failed:
                        note = f"Tracking {len(trackers)} tracker points"
                else:
                    note = "No pose detected. Adjust camera angles or lighting."

                now = time.perf_counter()
                fps = 1.0 / max(now - last_tick, 1e-6)
                last_tick = now

                self._emit_update(
                    TrackingUpdate(
                        fused_frame=fused_frame,
                        camera_previews=previews,
                        fused_preview=_render_fused_preview(fused_frame, note),
                        fps=fps,
                        note=note,
                        active_camera_count=len(runtimes),
                    )
                )
                time.sleep(0.001)
        except Exception as error:
            self._emit_update(TrackingUpdate(note=f"Tracking loop failed: {error}"))
        finally:
            for runtime in runtimes:
                runtime.close()
