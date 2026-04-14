from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Callable


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DOWNLOADS_DIR = PROJECT_ROOT / ".downloads"
LOGS_DIR = PROJECT_ROOT / ".logs"
VMT_RELEASE_API = "https://api.github.com/repos/gpsnmeajp/VirtualMotionTracker/releases/latest"
OPENVR_PATHS_FILE = Path(os.environ.get("LOCALAPPDATA", "")) / "openvr" / "openvrpaths.vrpath"
DEFAULT_VMT_INSTALL_DIR = Path("C:/vmt_driver")
DEFAULT_VMT_MANAGER_PATH = DEFAULT_VMT_INSTALL_DIR / "vmt_manager" / "vmt_manager.exe"
DEFAULT_VMT_MANAGER_ALT_PATH = DEFAULT_VMT_INSTALL_DIR / "VMTManager" / "VMTManager.exe"
DEFAULT_HEADLESS_HMD_BUILD_DIR = PROJECT_ROOT / "steamvr" / "ai_headless_hmd_driver" / "build" / "output" / "aitrackinghmd"
POSE_MODEL_URLS = {
    "lite": (
        "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
        "pose_landmarker_lite/float16/1/pose_landmarker_lite.task"
    ),
    "full": (
        "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
        "pose_landmarker_full/float16/1/pose_landmarker_full.task"
    ),
    "heavy": (
        "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
        "pose_landmarker_heavy/float16/1/pose_landmarker_heavy.task"
    ),
}


@dataclass(frozen=True)
class SteamVrStatus:
    vmt_manager_path: Path | None
    vmt_running: bool
    bridge_running: bool
    bridge_port_open: bool
    hmd_driver_built: bool
    hmd_driver_registered: bool
    hmd_port_open: bool


def _is_windows() -> bool:
    return sys.platform.startswith("win")


def _process_exists(image_name: str) -> bool:
    if not _is_windows():
        return False
    result = subprocess.run(
        ["tasklist", "/FI", f"IMAGENAME eq {image_name}"],
        capture_output=True,
        text=True,
        check=False,
    )
    return image_name.lower() in result.stdout.lower()


def _is_udp_port_open(port: int) -> bool:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(("127.0.0.1", port))
        return False
    except OSError:
        return True
    finally:
        probe.close()


def _find_udp_listener_pid(port: int) -> int | None:
    if not _is_windows():
        return None
    result = subprocess.run(
        ["netstat", "-ano", "-p", "UDP"],
        capture_output=True,
        text=True,
        check=False,
    )
    target = f":{port}"
    for line in result.stdout.splitlines():
        if target not in line:
            continue
        parts = line.split()
        if len(parts) < 4:
            continue
        try:
            return int(parts[-1])
        except ValueError:
            continue
    return None


def _terminate_process(pid: int) -> None:
    if pid <= 0:
        return
    subprocess.run(
        ["taskkill", "/PID", str(pid), "/F"],
        capture_output=True,
        text=True,
        check=False,
    )


def _download_json(url: str) -> dict:
    request = urllib.request.Request(url, headers={"User-Agent": "AI-Virtual-Reality"})
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.loads(response.read().decode("utf-8"))


def _download_file(url: str, target: Path) -> Path:
    target.parent.mkdir(parents=True, exist_ok=True)
    request = urllib.request.Request(url, headers={"User-Agent": "AI-Virtual-Reality"})
    with urllib.request.urlopen(request, timeout=120) as response:
        data = response.read()
    target.write_bytes(data)
    return target


def _load_openvr_paths() -> dict:
    if not OPENVR_PATHS_FILE.exists():
        return {}
    try:
        return json.loads(OPENVR_PATHS_FILE.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def find_vmt_manager_path() -> Path | None:
    candidates = [
        DEFAULT_VMT_MANAGER_PATH,
        DEFAULT_VMT_MANAGER_ALT_PATH,
        Path(os.environ.get("ProgramFiles", "")) / "Virtual Motion Tracker" / "vmt_manager.exe",
        Path(os.environ.get("ProgramFiles(x86)", "")) / "Virtual Motion Tracker" / "vmt_manager.exe",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_headless_hmd_build_path() -> Path | None:
    return DEFAULT_HEADLESS_HMD_BUILD_DIR if DEFAULT_HEADLESS_HMD_BUILD_DIR.exists() else None


def is_headless_hmd_registered() -> bool:
    paths = _load_openvr_paths()
    external = paths.get("external_drivers", [])
    if not isinstance(external, list):
        return False
    target = str(DEFAULT_HEADLESS_HMD_BUILD_DIR).lower().replace("/", "\\")
    for item in external:
        if not isinstance(item, str):
            continue
        normalized = item.lower().replace("/", "\\")
        if normalized == target:
            return True
    return False


def infer_pose_model_variant(model_path: str) -> str:
    lowered = model_path.replace("\\", "/").lower()
    if "pose_landmarker_lite" in lowered:
        return "lite"
    if "pose_landmarker_heavy" in lowered:
        return "heavy"
    return "full"


def ensure_pose_model(
    progress: Callable[[str], None],
    relative_path: str = "models/pose_landmarker_full.task",
    variant: str | None = None,
) -> Path:
    target = Path(relative_path)
    if not target.is_absolute():
        target = PROJECT_ROOT / target
    selected_variant = (variant or infer_pose_model_variant(str(target))).strip().lower()
    if selected_variant not in POSE_MODEL_URLS:
        raise RuntimeError(f"Unknown pose model variant: {selected_variant}")
    if target.exists():
        progress(f"Pose model ready: {target}")
        return target
    progress(f"Downloading MediaPipe pose model ({selected_variant})...")
    return _download_file(POSE_MODEL_URLS[selected_variant], target)


def ensure_vmt_installed(progress: Callable[[str], None]) -> Path:
    if not _is_windows():
        raise RuntimeError("VMT auto-setup is only supported on Windows.")

    existing = find_vmt_manager_path()
    if existing is not None:
        progress(f"VMT found: {existing}")
        return existing

    progress("Downloading latest Virtual Motion Tracker installer...")
    release = _download_json(VMT_RELEASE_API)
    assets = release.get("assets", [])
    installer = next((asset for asset in assets if asset.get("name", "").endswith(".exe")), None)
    if installer is None:
        raise RuntimeError("Could not find a VMT installer in the latest GitHub release.")

    DOWNLOADS_DIR.mkdir(parents=True, exist_ok=True)
    installer_path = DOWNLOADS_DIR / installer["name"]
    _download_file(installer["browser_download_url"], installer_path)

    progress("Running VMT installer silently...")
    subprocess.run(
        [
            str(installer_path),
            "/VERYSILENT",
            "/SUPPRESSMSGBOXES",
            "/NORESTART",
            "/SP-",
        ],
        check=True,
        cwd=str(PROJECT_ROOT),
    )

    installed = find_vmt_manager_path()
    if installed is None:
        raise RuntimeError("VMT installer finished, but VMTManager.exe was not found.")
    progress(f"VMT installed: {installed}")
    return installed


def launch_vmt_manager(progress: Callable[[str], None]) -> Path:
    manager_path = ensure_vmt_installed(progress)
    if _process_exists("vmt_manager.exe"):
        progress("VMT Manager is already running.")
        return manager_path

    subprocess.Popen([str(manager_path)], cwd=str(manager_path.parent))
    progress("VMT Manager launched.")
    return manager_path


def launch_bridge(
    progress: Callable[[str], None],
    listen_port: int = 7000,
    hmd_port: int = 39575,
    enable_hmd: bool = True,
    enable_controllers: bool = True,
    x_axis_scale: float = 1.0,
    y_axis_scale: float = 1.0,
    z_axis_scale: float = -1.0,
    hmd_y_offset: float = 0.0,
    controller_position_scale: float = 2.2,
) -> Path:
    if _is_udp_port_open(listen_port):
        existing_pid = _find_udp_listener_pid(listen_port)
        if existing_pid is not None:
            progress(f"Restarting bridge on UDP {listen_port} (PID {existing_pid})...")
            _terminate_process(existing_pid)
            time.sleep(0.6)

    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOGS_DIR / "vmt_bridge.log"
    with log_path.open("a", encoding="utf-8") as handle:
        subprocess.Popen(
            [
                sys.executable,
                str(PROJECT_ROOT / "tools" / "vmt_bridge.py"),
                "--listen-port",
                str(listen_port),
                "--hmd-port",
                str(hmd_port),
                *( [] if enable_hmd else ["--disable-hmd"] ),
                *( [] if enable_controllers else ["--disable-controllers"] ),
                "--x-axis-scale",
                str(x_axis_scale),
                "--y-axis-scale",
                str(y_axis_scale),
                "--z-axis-scale",
                str(z_axis_scale),
                "--hmd-y-offset",
                str(hmd_y_offset),
                "--controller-position-scale",
                str(controller_position_scale),
            ],
            cwd=str(PROJECT_ROOT),
            stdout=handle,
            stderr=subprocess.STDOUT,
        )
    progress(f"Bridge launched on UDP {listen_port}.")
    return log_path


def get_steamvr_status(listen_port: int = 7000, hmd_port: int = 39575) -> SteamVrStatus:
    return SteamVrStatus(
        vmt_manager_path=find_vmt_manager_path(),
        vmt_running=_process_exists("vmt_manager.exe"),
        bridge_running=_process_exists("python.exe") and _is_udp_port_open(listen_port),
        bridge_port_open=_is_udp_port_open(listen_port),
        hmd_driver_built=find_headless_hmd_build_path() is not None,
        hmd_driver_registered=is_headless_hmd_registered(),
        hmd_port_open=_is_udp_port_open(hmd_port),
    )
