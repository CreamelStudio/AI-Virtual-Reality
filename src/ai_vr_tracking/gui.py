from __future__ import annotations

import queue
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Dict, List

from .config import load_config, save_config
from .models import AppConfig, CameraConfig, OutputConfig, TrackingConfig, TrackingMode, TrackingUpdate
from .output import TrackingOutputRouter
from .tracking import TrackingEngine, detect_missing_runtime_dependencies, list_available_cameras


class CameraRow:
    def __init__(self, parent: ttk.Frame, config: CameraConfig) -> None:
        self.config = config
        self.enabled_var = tk.BooleanVar(value=config.enabled)
        self.weight_var = tk.DoubleVar(value=config.weight)
        self.mirror_var = tk.BooleanVar(value=config.mirror)

        self.frame = ttk.Frame(parent)
        self.frame.columnconfigure(1, weight=1)

        ttk.Checkbutton(self.frame, variable=self.enabled_var).grid(row=0, column=0, padx=(0, 6))
        ttk.Label(self.frame, text=f"Camera {config.camera_index}").grid(row=0, column=1, sticky="w")

        ttk.Label(self.frame, text="Weight").grid(row=0, column=2, padx=(10, 4))
        ttk.Spinbox(
            self.frame,
            from_=0.2,
            to=3.0,
            increment=0.1,
            width=5,
            textvariable=self.weight_var,
        ).grid(row=0, column=3, padx=(0, 8))

        ttk.Checkbutton(self.frame, text="Mirror", variable=self.mirror_var).grid(row=0, column=4)

    def as_config(self) -> CameraConfig:
        return CameraConfig(
            camera_index=self.config.camera_index,
            enabled=self.enabled_var.get(),
            weight=float(self.weight_var.get()),
            mirror=self.mirror_var.get(),
        )


class TrackingGuiApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("AI Body Tracking VR Bridge")
        self.geometry("1460x920")
        self.minsize(1180, 760)
        self.configure(bg="#10131a")

        self.style = ttk.Style(self)
        self.style.theme_use("clam")
        self.style.configure(".", background="#10131a", foreground="#eff4ff", fieldbackground="#1b2330")
        self.style.configure("TFrame", background="#10131a")
        self.style.configure("TLabelframe", background="#10131a", foreground="#f4f7ff")
        self.style.configure("TLabelframe.Label", background="#10131a", foreground="#f4f7ff")
        self.style.configure("TLabel", background="#10131a", foreground="#d6deed")
        self.style.configure("TButton", background="#2f74ff", foreground="#ffffff", padding=8)
        self.style.configure("TCheckbutton", background="#10131a", foreground="#d6deed")
        self.style.configure("TCombobox", padding=5)

        self.config_model = load_config()
        self.output_router = TrackingOutputRouter(self.config_model.output)
        self.tracking_engine: TrackingEngine | None = None
        self.update_queue: "queue.Queue[TrackingUpdate]" = queue.Queue(maxsize=2)
        self.preview_refs: Dict[str, object] = {}
        self.camera_rows: List[CameraRow] = []
        self.camera_preview_labels: Dict[int, ttk.Label] = {}

        self.mode_var = tk.StringVar(value=self.config_model.tracking.mode.value)
        self.min_visibility_var = tk.DoubleVar(value=self.config_model.tracking.min_visibility)
        self.smoothing_var = tk.DoubleVar(value=self.config_model.tracking.smoothing)
        self.output_enabled_var = tk.BooleanVar(value=self.config_model.output.enabled)
        self.protocol_var = tk.StringVar(value=self.config_model.output.protocol)
        self.host_var = tk.StringVar(value=self.config_model.output.host)
        self.port_var = tk.StringVar(value=str(self.config_model.output.port))
        self.emit_joints_var = tk.BooleanVar(value=self.config_model.output.emit_joints)

        self.status_var = tk.StringVar(value="Ready")
        self.runtime_var = tk.StringVar(value="Press Start Tracking when cameras are configured.")

        self._build_layout()
        self._rebuild_camera_rows(
            list_available_cameras(self.config_model.tracking.max_cameras_to_scan)
            or [camera.camera_index for camera in self.config_model.cameras]
            or [0]
        )
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(60, self._poll_updates)

    def _build_layout(self) -> None:
        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        controls = ttk.Frame(self, padding=16)
        controls.grid(row=0, column=0, sticky="nsew")
        controls.columnconfigure(0, weight=1)

        previews = ttk.Frame(self, padding=(0, 16, 16, 16))
        previews.grid(row=0, column=1, sticky="nsew")
        previews.columnconfigure(0, weight=1)
        previews.rowconfigure(0, weight=1)
        previews.rowconfigure(1, weight=1)

        tracking_box = ttk.LabelFrame(controls, text="Tracking", padding=14)
        tracking_box.grid(row=0, column=0, sticky="ew")
        tracking_box.columnconfigure(1, weight=1)

        ttk.Label(tracking_box, text="Mode").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            tracking_box,
            state="readonly",
            values=[TrackingMode.FULL_BODY.value, TrackingMode.UPPER_BODY.value],
            textvariable=self.mode_var,
        ).grid(row=0, column=1, sticky="ew")

        ttk.Label(tracking_box, text="Min visibility").grid(row=1, column=0, sticky="w", pady=(10, 0))
        ttk.Scale(
            tracking_box,
            variable=self.min_visibility_var,
            from_=0.2,
            to=0.95,
            orient="horizontal",
        ).grid(row=1, column=1, sticky="ew", pady=(10, 0))

        ttk.Label(tracking_box, text="Smoothing").grid(row=2, column=0, sticky="w", pady=(10, 0))
        ttk.Scale(
            tracking_box,
            variable=self.smoothing_var,
            from_=0.0,
            to=0.9,
            orient="horizontal",
        ).grid(row=2, column=1, sticky="ew", pady=(10, 0))

        output_box = ttk.LabelFrame(controls, text="Output Bridge", padding=14)
        output_box.grid(row=1, column=0, sticky="ew", pady=(14, 0))
        output_box.columnconfigure(1, weight=1)

        ttk.Checkbutton(output_box, text="Enable output", variable=self.output_enabled_var).grid(
            row=0, column=0, columnspan=2, sticky="w"
        )

        ttk.Label(output_box, text="Protocol").grid(row=1, column=0, sticky="w", pady=(10, 0))
        ttk.Combobox(
            output_box,
            state="readonly",
            values=["osc", "json_udp", "vmt_osc"],
            textvariable=self.protocol_var,
        ).grid(row=1, column=1, sticky="ew", pady=(10, 0))

        ttk.Label(output_box, text="Host").grid(row=2, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(output_box, textvariable=self.host_var).grid(row=2, column=1, sticky="ew", pady=(10, 0))

        ttk.Label(output_box, text="Port").grid(row=3, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(output_box, textvariable=self.port_var).grid(row=3, column=1, sticky="ew", pady=(10, 0))

        ttk.Checkbutton(output_box, text="Emit joint stream", variable=self.emit_joints_var).grid(
            row=4, column=0, columnspan=2, sticky="w", pady=(10, 0)
        )

        camera_box = ttk.LabelFrame(controls, text="Cameras", padding=14)
        camera_box.grid(row=2, column=0, sticky="ew", pady=(14, 0))
        camera_box.columnconfigure(0, weight=1)
        self.camera_rows_container = ttk.Frame(camera_box)
        self.camera_rows_container.grid(row=0, column=0, sticky="ew")

        actions = ttk.Frame(camera_box)
        actions.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        actions.columnconfigure(0, weight=1)
        ttk.Button(actions, text="Rescan Cameras", command=self._rescan_cameras).grid(row=0, column=0, sticky="ew")

        control_buttons = ttk.Frame(controls)
        control_buttons.grid(row=3, column=0, sticky="ew", pady=(14, 0))
        control_buttons.columnconfigure(0, weight=1)
        control_buttons.columnconfigure(1, weight=1)
        ttk.Button(control_buttons, text="Start Tracking", command=self._start_tracking).grid(
            row=0, column=0, sticky="ew", padx=(0, 6)
        )
        ttk.Button(control_buttons, text="Stop Tracking", command=self._stop_tracking).grid(
            row=0, column=1, sticky="ew", padx=(6, 0)
        )

        status_box = ttk.LabelFrame(controls, text="Status", padding=14)
        status_box.grid(row=4, column=0, sticky="nsew", pady=(14, 0))
        status_box.columnconfigure(0, weight=1)
        controls.rowconfigure(4, weight=1)
        ttk.Label(status_box, textvariable=self.status_var, wraplength=320, justify="left").grid(
            row=0, column=0, sticky="nw"
        )
        ttk.Label(status_box, textvariable=self.runtime_var, wraplength=320, justify="left").grid(
            row=1, column=0, sticky="nw", pady=(8, 0)
        )

        fused_box = ttk.LabelFrame(previews, text="Fused View", padding=10)
        fused_box.grid(row=0, column=0, sticky="nsew")
        fused_box.columnconfigure(0, weight=1)
        fused_box.rowconfigure(0, weight=1)
        self.fused_preview_label = ttk.Label(fused_box, text="No fused preview yet", anchor="center")
        self.fused_preview_label.grid(row=0, column=0, sticky="nsew")

        multi_box = ttk.LabelFrame(previews, text="Camera Views", padding=10)
        multi_box.grid(row=1, column=0, sticky="nsew", pady=(14, 0))
        for row in range(2):
            multi_box.rowconfigure(row, weight=1)
        for column in range(2):
            multi_box.columnconfigure(column, weight=1)

        for slot in range(4):
            frame = ttk.Frame(multi_box, padding=4)
            frame.grid(row=slot // 2, column=slot % 2, sticky="nsew")
            frame.columnconfigure(0, weight=1)
            frame.rowconfigure(0, weight=1)
            label = ttk.Label(frame, text="Camera preview", anchor="center")
            label.grid(row=0, column=0, sticky="nsew")
            self.camera_preview_labels[slot] = label

    def _rebuild_camera_rows(self, camera_indices: List[int]) -> None:
        saved_by_index = {camera.camera_index: camera for camera in self.config_model.cameras}
        for child in self.camera_rows_container.winfo_children():
            child.destroy()
        self.camera_rows.clear()

        for row_index, camera_index in enumerate(sorted(set(camera_indices))):
            config = saved_by_index.get(camera_index, CameraConfig(camera_index=camera_index, enabled=(row_index == 0)))
            row = CameraRow(self.camera_rows_container, config)
            row.frame.grid(row=row_index, column=0, sticky="ew", pady=4)
            self.camera_rows.append(row)

        if not self.camera_rows:
            row = CameraRow(self.camera_rows_container, CameraConfig(camera_index=0, enabled=True))
            row.frame.grid(row=0, column=0, sticky="ew", pady=4)
            self.camera_rows.append(row)

    def _rescan_cameras(self) -> None:
        missing = detect_missing_runtime_dependencies()
        if "opencv-python" in missing:
            self.status_var.set("opencv-python is required before cameras can be scanned.")
            return
        cameras = list_available_cameras(self.config_model.tracking.max_cameras_to_scan)
        self._rebuild_camera_rows(cameras or [0])
        self.status_var.set(f"Camera scan complete. Found: {cameras or ['fallback camera 0']}")

    def _collect_config(self) -> AppConfig:
        tracking = TrackingConfig(
            mode=TrackingMode(self.mode_var.get()),
            min_visibility=float(self.min_visibility_var.get()),
            smoothing=float(self.smoothing_var.get()),
            output_fps=self.config_model.tracking.output_fps,
            model_complexity=self.config_model.tracking.model_complexity,
            max_cameras_to_scan=self.config_model.tracking.max_cameras_to_scan,
        )
        output = OutputConfig(
            enabled=self.output_enabled_var.get(),
            protocol=self.protocol_var.get(),
            host=self.host_var.get().strip() or "127.0.0.1",
            port=int(self.port_var.get()),
            emit_joints=self.emit_joints_var.get(),
        )
        cameras = [row.as_config() for row in self.camera_rows]
        return AppConfig(tracking=tracking, output=output, cameras=cameras)

    def _start_tracking(self) -> None:
        missing = detect_missing_runtime_dependencies()
        if missing:
            messagebox.showerror(
                "Missing dependencies",
                "Install required packages first:\n\npip install -r requirements.txt\n\nMissing: "
                + ", ".join(missing),
            )
            return

        try:
            config = self._collect_config()
        except ValueError:
            messagebox.showerror("Invalid configuration", "Port and numeric settings must be valid numbers.")
            return

        if not any(camera.enabled for camera in config.cameras):
            messagebox.showwarning("No camera enabled", "Enable at least one webcam before starting.")
            return

        self._stop_tracking()
        self.config_model = config
        self.output_router.update_config(config.output)
        self.tracking_engine = TrackingEngine(config, self.output_router)
        self.tracking_engine.start(self._push_update)
        self.status_var.set("Tracking engine started.")
        self.runtime_var.set("Waiting for camera frames...")
        save_config(self.config_model)

    def _stop_tracking(self) -> None:
        if self.tracking_engine is not None:
            self.tracking_engine.stop()
            self.tracking_engine = None
        self.status_var.set("Tracking stopped.")

    def _push_update(self, update: TrackingUpdate) -> None:
        try:
            if self.update_queue.full():
                self.update_queue.get_nowait()
            self.update_queue.put_nowait(update)
        except queue.Full:
            pass

    def _render_image(self, target: ttk.Label, image, key: str, max_size: tuple[int, int]) -> None:
        if image is None:
            target.configure(text="No preview", image="")
            return

        try:
            import cv2
            from PIL import Image, ImageTk
        except ImportError:
            target.configure(text="Preview requires Pillow and OpenCV", image="")
            return

        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(rgb_image)
        pil_image.thumbnail(max_size)
        tk_image = ImageTk.PhotoImage(pil_image)
        self.preview_refs[key] = tk_image
        target.configure(image=tk_image, text="")

    def _poll_updates(self) -> None:
        latest = None
        while True:
            try:
                latest = self.update_queue.get_nowait()
            except queue.Empty:
                break

        if latest is not None:
            if latest.fused_preview is not None:
                self._render_image(self.fused_preview_label, latest.fused_preview, "fused", (860, 520))

            for slot, label in self.camera_preview_labels.items():
                target_camera = sorted(latest.camera_previews)[slot] if slot < len(latest.camera_previews) else None
                if target_camera is None:
                    label.configure(text="Camera preview", image="")
                    continue
                image = latest.camera_previews[target_camera]
                self._render_image(label, image, f"camera-{target_camera}", (420, 240))

            tracker_count = len(latest.fused_frame.trackers) if latest.fused_frame is not None else 0
            self.status_var.set(
                f"{latest.note}\nFPS: {latest.fps:.1f} | Active cameras: {latest.active_camera_count} | "
                f"Tracker points: {tracker_count}"
            )
            self.runtime_var.set(
                f"Mode: {self.mode_var.get()} | Output: {self.protocol_var.get()} -> "
                f"{self.host_var.get()}:{self.port_var.get()}"
            )

        self.after(60, self._poll_updates)

    def _on_close(self) -> None:
        try:
            self._stop_tracking()
            self.config_model = self._collect_config()
            save_config(self.config_model)
        except Exception:
            pass
        self.output_router.close()
        self.destroy()


def main() -> None:
    app = TrackingGuiApp()
    app.mainloop()
