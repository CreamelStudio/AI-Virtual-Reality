from __future__ import annotations

import threading
import time
import sys
from pathlib import Path
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


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_POSE_MODEL_CANDIDATES = (
    Path("models/pose_landmarker_full.task"),
    Path("models/pose_landmarker_heavy.task"),
    Path("models/pose_landmarker.task"),
    Path("pose_landmarker.task"),
)


def detect_missing_runtime_dependencies(config: AppConfig | None = None) -> List[str]:
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
    if "mediapipe" not in missing:
        try:
            _get_pose_backend(config)
        except RuntimeError as error:
            missing.append(str(error))
    return missing


def _open_video_capture(camera_index: int) -> Any:
    import cv2

    if sys.platform.startswith("win"):
        return cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    return cv2.VideoCapture(camera_index)


def _resolve_model_candidate(path_text: str) -> Path:
    path = Path(path_text)
    if path.is_absolute():
        return path
    return PROJECT_ROOT / path


def _find_pose_model_path(config: AppConfig | None = None) -> Path | None:
    candidates: List[Path] = []
    if config is not None:
        configured = config.tracking.pose_model_path.strip()
        if configured:
            candidates.append(_resolve_model_candidate(configured))

    candidates.extend(PROJECT_ROOT / candidate for candidate in DEFAULT_POSE_MODEL_CANDIDATES)

    seen: set[Path] = set()
    for candidate in candidates:
        normalized = candidate.resolve(strict=False)
        if normalized in seen:
            continue
        seen.add(normalized)
        if candidate.exists() and candidate.is_file():
            return candidate
    return None


def list_available_cameras(max_index: int = 5) -> List[int]:
    try:
        import cv2
    except ImportError:
        return []

    cameras: List[int] = []
    for index in range(max_index + 1):
        capture = _open_video_capture(index)
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


def _get_pose_backend(config: AppConfig | None = None) -> Dict[str, Any]:
    import mediapipe as mp

    solutions = getattr(mp, "solutions", None)
    pose_module = getattr(solutions, "pose", None) if solutions is not None else None
    if pose_module is not None:
        return {"kind": "solutions", "mp": mp, "pose_module": pose_module}

    tasks = getattr(mp, "tasks", None)
    vision = getattr(tasks, "vision", None) if tasks is not None else None
    pose_landmarker = getattr(vision, "PoseLandmarker", None) if vision is not None else None
    pose_options = getattr(vision, "PoseLandmarkerOptions", None) if vision is not None else None
    running_mode = getattr(vision, "RunningMode", None) if vision is not None else None
    base_options = getattr(tasks, "BaseOptions", None) if tasks is not None else None

    if all(item is not None for item in (pose_landmarker, pose_options, running_mode, base_options)):
        model_path = _find_pose_model_path(config)
        if model_path is None:
            configured = config.tracking.pose_model_path if config is not None else "models/pose_landmarker_full.task"
            raise RuntimeError(
                "MediaPipe Tasks is available, but no pose model file was found. "
                f"Put `pose_landmarker_full.task` at `{configured}` or update `tracking.pose_model_path` in config.json."
            )
        return {
            "kind": "tasks",
            "mp": mp,
            "pose_landmarker": pose_landmarker,
            "pose_options": pose_options,
            "running_mode": running_mode,
            "base_options": base_options,
            "model_path": str(model_path),
        }

    version = getattr(mp, "__version__", "unknown")
    raise RuntimeError(
        "Incompatible mediapipe install detected "
        f"(version {version}). Neither `mediapipe.solutions.pose` nor `mediapipe.tasks.vision.PoseLandmarker` "
        "is available in the current environment."
    )


def _draw_pose_preview(preview: Any, joints: Dict[str, JointSample]) -> None:
    import cv2

    if not joints:
        return

    height, width = preview.shape[:2]

    def project(sample: JointSample) -> Tuple[int, int]:
        return int(sample.x * width), int(sample.y * height)

    active_joints = set(joints)
    for start, end in POSE_CONNECTIONS:
        if start not in active_joints or end not in active_joints:
            continue
        cv2.line(preview, project(joints[start]), project(joints[end]), (93, 110, 140), 2)

    for joint in joints.values():
        cv2.circle(preview, project(joint), max(2, int(3 + (joint.visibility * 2))), (221, 227, 236), -1)


@dataclass
class _CameraRuntime:
    config: CameraConfig
    capture: Any
    pose: Any
    backend_kind: str

    def read(self) -> Tuple[PoseObservation | None, Any | None]:
        import cv2

        ok, frame = self.capture.read()
        if not ok:
            return None, None

        if self.config.mirror:
            frame = cv2.flip(frame, 1)

        preview = frame.copy()
        if self.backend_kind == "solutions":
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.pose.process(rgb_frame)
            observation = _legacy_results_to_observation(results, self.config)
        else:
            import mediapipe as mp

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            results = self.pose.detect_for_video(mp_image, int(time.perf_counter() * 1000))
            observation = _task_results_to_observation(results, self.config)

        if observation is not None:
            _draw_pose_preview(preview, observation.joints)

        label = f"Cam {self.config.camera_index} {'OK' if observation else 'No pose'}"
        color = (80, 220, 140) if observation else (120, 130, 150)
        cv2.putText(preview, label, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        return observation, preview

    def close(self) -> None:
        self.capture.release()
        self.pose.close()


def _landmarks_to_observation(
    config: CameraConfig,
    normalized_landmarks: List[Any],
    world_landmarks: List[Any] | None = None,
) -> PoseObservation | None:
    if not normalized_landmarks:
        return None

    world_landmarks = world_landmarks or []
    joints: Dict[str, JointSample] = {}
    visibilities = []
    for index, (name, landmark) in enumerate(zip(LANDMARK_NAMES, normalized_landmarks)):
        world_landmark = world_landmarks[index] if index < len(world_landmarks) else None
        visibility = float(getattr(landmark, "visibility", getattr(world_landmark, "visibility", 1.0)))
        visibilities.append(visibility)
        joints[name] = JointSample(
            name=name,
            x=float(landmark.x),
            y=float(landmark.y),
            z=float(getattr(world_landmark, "z", landmark.z)),
            visibility=visibility,
        )

    score = sum(visibilities) / max(len(visibilities), 1)
    return PoseObservation(
        camera_index=config.camera_index,
        camera_weight=config.weight,
        joints=joints,
        score=score,
    )


def _legacy_results_to_observation(results: Any, config: CameraConfig) -> PoseObservation | None:
    if results.pose_landmarks is None:
        return None

    normalized_landmarks = list(results.pose_landmarks.landmark)
    world_landmarks = list(results.pose_world_landmarks.landmark) if results.pose_world_landmarks is not None else []
    return _landmarks_to_observation(config, normalized_landmarks, world_landmarks)


def _task_results_to_observation(results: Any, config: CameraConfig) -> PoseObservation | None:
    pose_landmarks = getattr(results, "pose_landmarks", None) or []
    if not pose_landmarks:
        return None

    normalized_landmarks = list(pose_landmarks[0])
    world_sets = getattr(results, "pose_world_landmarks", None) or []
    world_landmarks = list(world_sets[0]) if world_sets else []
    return _landmarks_to_observation(config, normalized_landmarks, world_landmarks)


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

        backend = _get_pose_backend(self._config)

        active_cameras = [camera for camera in self._config.cameras if camera.enabled]
        runtimes: List[_CameraRuntime] = []

        for camera in active_cameras:
            capture = _open_video_capture(camera.camera_index)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
            if not capture.isOpened():
                capture.release()
                continue

            if backend["kind"] == "solutions":
                pose = backend["pose_module"].Pose(
                    static_image_mode=False,
                    model_complexity=self._config.tracking.model_complexity,
                    smooth_landmarks=True,
                    enable_segmentation=False,
                    min_detection_confidence=max(0.3, self._config.tracking.min_visibility - 0.1),
                    min_tracking_confidence=max(0.3, self._config.tracking.min_visibility - 0.1),
                )
            else:
                pose = backend["pose_landmarker"].create_from_options(
                    backend["pose_options"](
                        base_options=backend["base_options"](model_asset_path=backend["model_path"]),
                        running_mode=backend["running_mode"].VIDEO,
                        num_poses=1,
                        min_pose_detection_confidence=max(0.3, self._config.tracking.min_visibility - 0.1),
                        min_pose_presence_confidence=max(0.3, self._config.tracking.min_visibility - 0.1),
                        min_tracking_confidence=max(0.3, self._config.tracking.min_visibility - 0.1),
                        output_segmentation_masks=False,
                    )
                )
            runtimes.append(
                _CameraRuntime(
                    config=camera,
                    capture=capture,
                    pose=pose,
                    backend_kind=backend["kind"],
                )
            )
        return runtimes

    def _run(self) -> None:
        missing = detect_missing_runtime_dependencies(self._config)
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
