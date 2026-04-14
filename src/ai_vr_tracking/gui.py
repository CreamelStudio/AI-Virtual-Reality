from __future__ import annotations

import queue
import tkinter as tk
import threading
import math
from tkinter import messagebox, ttk
from typing import Dict, List

from .config import load_config, save_config
from .models import AppConfig, CameraConfig, OutputConfig, TrackingConfig, TrackingMode, TrackingUpdate
from .output import TrackingOutputRouter
from .steamvr_setup import ensure_pose_model, get_steamvr_status, infer_pose_model_variant, launch_bridge, launch_vmt_manager
from .tracking import TrackingEngine, detect_missing_runtime_dependencies, list_available_cameras


class CameraRow:
    def __init__(self, parent: ttk.Frame, config: CameraConfig) -> None:
        self.config = config
        self.enabled_var = tk.BooleanVar(value=config.enabled)
        self.weight_var = tk.DoubleVar(value=config.weight)
        self.mirror_var = tk.BooleanVar(value=config.mirror)
        self.position_x_var = tk.DoubleVar(value=config.position_x)
        self.position_y_var = tk.DoubleVar(value=config.position_y)
        self.position_z_var = tk.DoubleVar(value=config.position_z)
        self.yaw_var = tk.DoubleVar(value=config.yaw_degrees)
        self.pitch_var = tk.DoubleVar(value=config.pitch_degrees)
        self.roll_var = tk.DoubleVar(value=config.roll_degrees)

        self.frame = ttk.Frame(parent)
        self.frame.columnconfigure(1, weight=1)
        self.frame.columnconfigure(8, weight=1)

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

        ttk.Label(self.frame, text="Pos X/Y/Z").grid(row=1, column=1, sticky="w", pady=(6, 0))
        ttk.Entry(self.frame, width=6, textvariable=self.position_x_var).grid(row=1, column=2, pady=(6, 0))
        ttk.Entry(self.frame, width=6, textvariable=self.position_y_var).grid(row=1, column=3, pady=(6, 0))
        ttk.Entry(self.frame, width=6, textvariable=self.position_z_var).grid(row=1, column=4, pady=(6, 0))

        ttk.Label(self.frame, text="Yaw/Pitch/Roll").grid(row=1, column=5, sticky="w", padx=(10, 4), pady=(6, 0))
        ttk.Entry(self.frame, width=6, textvariable=self.yaw_var).grid(row=1, column=6, pady=(6, 0))
        ttk.Entry(self.frame, width=6, textvariable=self.pitch_var).grid(row=1, column=7, pady=(6, 0))
        ttk.Entry(self.frame, width=6, textvariable=self.roll_var).grid(row=1, column=8, pady=(6, 0), sticky="w")

    def as_config(self) -> CameraConfig:
        return CameraConfig(
            camera_index=self.config.camera_index,
            enabled=self.enabled_var.get(),
            weight=float(self.weight_var.get()),
            mirror=self.mirror_var.get(),
            position_x=float(self.position_x_var.get()),
            position_y=float(self.position_y_var.get()),
            position_z=float(self.position_z_var.get()),
            yaw_degrees=float(self.yaw_var.get()),
            pitch_degrees=float(self.pitch_var.get()),
            roll_degrees=float(self.roll_var.get()),
        )

    def update_pose(self, *, x_value: float, y_value: float, z_value: float, yaw_value: float) -> None:
        self.position_x_var.set(round(x_value, 3))
        self.position_y_var.set(round(y_value, 3))
        self.position_z_var.set(round(z_value, 3))
        self.yaw_var.set(round(yaw_value, 1))


class CameraCalibrationDialog(tk.Toplevel):
    SCALE = 140.0
    ORIGIN_X = 420.0
    ORIGIN_Y = 280.0

    def __init__(self, parent: "TrackingGuiApp", camera_rows: List[CameraRow]) -> None:
        super().__init__(parent)
        self.title("Camera Calibration Position Setup")
        self.geometry("920x700")
        self.configure(bg="#10131a")
        self.transient(parent)
        self.grab_set()

        self._parent = parent
        self._camera_rows = camera_rows
        self._selected_index = 0
        self._drag_camera_index: int | None = None
        self._camera_ids: Dict[int, int] = {}
        self._label_ids: Dict[int, int] = {}

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        shell = ttk.Frame(self, padding=16)
        shell.grid(row=0, column=0, sticky="nsew")
        shell.columnconfigure(0, weight=1)
        shell.rowconfigure(1, weight=1)

        info = ttk.Label(
            shell,
            text=(
                "Drag cameras on the floor grid. Mouse wheel changes height (Y). "
                "Right-drag adjusts yaw. The text values update automatically."
            ),
            wraplength=860,
            justify="left",
        )
        info.grid(row=0, column=0, sticky="ew")

        self.canvas = tk.Canvas(shell, width=860, height=520, bg="#121722", highlightthickness=0)
        self.canvas.grid(row=1, column=0, sticky="nsew", pady=(12, 12))

        controls = ttk.Frame(shell)
        controls.grid(row=2, column=0, sticky="ew")
        controls.columnconfigure(1, weight=1)

        ttk.Label(controls, text="Selected camera").grid(row=0, column=0, sticky="w")
        self.camera_selector = ttk.Combobox(
            controls,
            state="readonly",
            values=[f"Camera {row.config.camera_index}" for row in self._camera_rows],
        )
        self.camera_selector.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        self.camera_selector.current(0)
        self.camera_selector.bind("<<ComboboxSelected>>", self._on_select_camera)

        self.coords_var = tk.StringVar(value="")
        ttk.Label(controls, textvariable=self.coords_var, justify="left").grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(10, 0)
        )
        ttk.Button(controls, text="Close", command=self.destroy).grid(row=0, column=2, rowspan=2, padx=(12, 0))

        self.canvas.bind("<Button-1>", self._on_left_down)
        self.canvas.bind("<B1-Motion>", self._on_left_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_left_up)
        self.canvas.bind("<Button-3>", self._on_right_down)
        self.canvas.bind("<B3-Motion>", self._on_right_drag)
        self.canvas.bind("<MouseWheel>", self._on_mouse_wheel)

        self._redraw()

    def _project(self, x_value: float, y_value: float, z_value: float) -> tuple[float, float]:
        screen_x = self.ORIGIN_X + ((x_value - z_value) * self.SCALE)
        screen_y = self.ORIGIN_Y + ((x_value + z_value) * self.SCALE * 0.5) - (y_value * self.SCALE)
        return screen_x, screen_y

    def _unproject_floor(self, screen_x: float, screen_y: float) -> tuple[float, float]:
        dx = (screen_x - self.ORIGIN_X) / self.SCALE
        dy = (screen_y - self.ORIGIN_Y) / (self.SCALE * 0.5)
        x_value = (dx + dy) / 2.0
        z_value = (dy - dx) / 2.0
        return x_value, z_value

    def _draw_grid(self) -> None:
        for step in range(-4, 5):
            start = self._project(-4, 0.0, step)
            end = self._project(4, 0.0, step)
            self.canvas.create_line(*start, *end, fill="#293246")
            start = self._project(step, 0.0, -4)
            end = self._project(step, 0.0, 4)
            self.canvas.create_line(*start, *end, fill="#293246")

        axis_origin = self._project(0.0, 0.0, 0.0)
        axis_x = self._project(1.2, 0.0, 0.0)
        axis_z = self._project(0.0, 0.0, 1.2)
        axis_y = self._project(0.0, 1.2, 0.0)
        self.canvas.create_line(*axis_origin, *axis_x, fill="#ff8b6b", width=3)
        self.canvas.create_line(*axis_origin, *axis_z, fill="#73d4ff", width=3)
        self.canvas.create_line(*axis_origin, *axis_y, fill="#7df0ae", width=3)
        self.canvas.create_text(axis_x[0] + 16, axis_x[1], text="X", fill="#ff8b6b")
        self.canvas.create_text(axis_z[0] + 16, axis_z[1], text="Z", fill="#73d4ff")
        self.canvas.create_text(axis_y[0], axis_y[1] - 16, text="Y", fill="#7df0ae")

    def _redraw(self) -> None:
        self.canvas.delete("all")
        self._draw_grid()
        self._camera_ids.clear()
        self._label_ids.clear()

        for row_index, camera_row in enumerate(self._camera_rows):
            x_value = float(camera_row.position_x_var.get())
            y_value = float(camera_row.position_y_var.get())
            z_value = float(camera_row.position_z_var.get())
            yaw_value = float(camera_row.yaw_var.get())
            screen_x, screen_y = self._project(x_value, y_value, z_value)
            radius = 12 if row_index == self._selected_index else 10
            fill = "#2f74ff" if row_index == self._selected_index else "#c5d4f7"
            outline = "#ffffff" if camera_row.enabled_var.get() else "#56637e"

            camera_id = self.canvas.create_oval(
                screen_x - radius,
                screen_y - radius,
                screen_x + radius,
                screen_y + radius,
                fill=fill,
                outline=outline,
                width=2,
            )
            self._camera_ids[camera_row.config.camera_index] = camera_id

            yaw_radians = math.radians(float(yaw_value))
            world_tip_x = x_value + (0.35 * math.sin(yaw_radians))
            world_tip_z = z_value - (0.35 * math.cos(yaw_radians))
            tip_x, tip_y = self._project(world_tip_x, y_value, world_tip_z)
            self.canvas.create_line(screen_x, screen_y, tip_x, tip_y, fill="#ffd166", width=3, arrow="last")

            label_id = self.canvas.create_text(
                screen_x,
                screen_y - 24,
                text=f"Cam {camera_row.config.camera_index}",
                fill="#eef3ff",
            )
            self._label_ids[camera_row.config.camera_index] = label_id

        self._update_selected_text()

    def _selected_row(self) -> CameraRow:
        return self._camera_rows[self._selected_index]

    def _update_selected_text(self) -> None:
        row = self._selected_row()
        self.coords_var.set(
            "Position "
            f"X={float(row.position_x_var.get()):.3f}, "
            f"Y={float(row.position_y_var.get()):.3f}, "
            f"Z={float(row.position_z_var.get()):.3f}    "
            f"Yaw={float(row.yaw_var.get()):.1f}"
        )

    def _find_camera_index_by_item(self, item_id: int) -> int | None:
        for row_index, camera_row in enumerate(self._camera_rows):
            if self._camera_ids.get(camera_row.config.camera_index) == item_id:
                return row_index
        return None

    def _on_select_camera(self, _event) -> None:
        self._selected_index = self.camera_selector.current()
        self._redraw()

    def _on_left_down(self, event) -> None:
        item_id = self.canvas.find_closest(event.x, event.y)
        if not item_id:
            return
        found_index = self._find_camera_index_by_item(item_id[0])
        if found_index is not None:
            self._selected_index = found_index
            self.camera_selector.current(found_index)
            self._drag_camera_index = found_index
            self._redraw()

    def _on_left_drag(self, event) -> None:
        if self._drag_camera_index is None:
            return
        row = self._camera_rows[self._drag_camera_index]
        x_value, z_value = self._unproject_floor(event.x, event.y + (float(row.position_y_var.get()) * self.SCALE))
        row.update_pose(
            x_value=x_value,
            y_value=float(row.position_y_var.get()),
            z_value=z_value,
            yaw_value=float(row.yaw_var.get()),
        )
        self._redraw()

    def _on_left_up(self, _event) -> None:
        self._drag_camera_index = None

    def _on_right_down(self, event) -> None:
        self._on_left_down(event)

    def _on_right_drag(self, event) -> None:
        row = self._selected_row()
        screen_x, screen_y = self._project(
            float(row.position_x_var.get()),
            float(row.position_y_var.get()),
            float(row.position_z_var.get()),
        )
        delta_x = event.x - screen_x
        delta_y = event.y - screen_y
        yaw_value = -(math.degrees(math.atan2(delta_x, delta_y)))
        row.update_pose(
            x_value=float(row.position_x_var.get()),
            y_value=float(row.position_y_var.get()),
            z_value=float(row.position_z_var.get()),
            yaw_value=yaw_value,
        )
        self._redraw()

    def _on_mouse_wheel(self, event) -> None:
        row = self._selected_row()
        delta = 0.05 if event.delta > 0 else -0.05
        row.update_pose(
            x_value=float(row.position_x_var.get()),
            y_value=float(row.position_y_var.get()) + delta,
            z_value=float(row.position_z_var.get()),
            yaw_value=float(row.yaw_var.get()),
        )
        self._redraw()


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
        self.bridge_enable_hmd_var = tk.BooleanVar(value=self.config_model.output.bridge_enable_hmd)
        self.bridge_enable_controllers_var = tk.BooleanVar(value=self.config_model.output.bridge_enable_controllers)
        self.bridge_x_axis_var = tk.DoubleVar(value=self.config_model.output.bridge_x_axis_scale)
        self.bridge_y_axis_var = tk.DoubleVar(value=self.config_model.output.bridge_y_axis_scale)
        self.bridge_z_axis_var = tk.DoubleVar(value=self.config_model.output.bridge_z_axis_scale)
        self.hmd_y_offset_var = tk.DoubleVar(value=self.config_model.output.hmd_y_offset)
        self.controller_position_scale_var = tk.DoubleVar(value=self.config_model.output.controller_position_scale)
        self.pose_model_path_var = tk.StringVar(value=self.config_model.tracking.pose_model_path)
        self.pose_model_variant_var = tk.StringVar(
            value=infer_pose_model_variant(self.config_model.tracking.pose_model_path)
        )

        self.status_var = tk.StringVar(value="Ready")
        self.runtime_var = tk.StringVar(value="Press Start Tracking when cameras are configured.")
        self.steamvr_var = tk.StringVar(value="SteamVR helper idle.")

        self._build_layout()
        self._rebuild_camera_rows(
            list_available_cameras(self.config_model.tracking.max_cameras_to_scan)
            or [camera.camera_index for camera in self.config_model.cameras]
            or [0]
        )
        self._refresh_steamvr_status()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(60, self._poll_updates)

    def _build_layout(self) -> None:
        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        controls_shell = ttk.Frame(self, padding=(16, 16, 8, 16))
        controls_shell.grid(row=0, column=0, sticky="nsew")
        controls_shell.columnconfigure(0, weight=1)
        controls_shell.rowconfigure(0, weight=1)

        self.controls_canvas = tk.Canvas(
            controls_shell,
            bg="#10131a",
            highlightthickness=0,
            borderwidth=0,
            width=420,
        )
        self.controls_canvas.grid(row=0, column=0, sticky="nsew")
        controls_scrollbar = ttk.Scrollbar(controls_shell, orient="vertical", command=self.controls_canvas.yview)
        controls_scrollbar.grid(row=0, column=1, sticky="ns")
        self.controls_canvas.configure(yscrollcommand=controls_scrollbar.set)

        controls = ttk.Frame(self.controls_canvas, padding=(0, 0, 8, 0))
        controls.columnconfigure(0, weight=1)
        self.controls_window_id = self.controls_canvas.create_window((0, 0), window=controls, anchor="nw")
        controls.bind("<Configure>", self._on_controls_configure)
        self.controls_canvas.bind("<Configure>", self._on_controls_canvas_configure)
        self.controls_canvas.bind_all("<MouseWheel>", self._on_mousewheel)

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

        ttk.Label(tracking_box, text="Pose model").grid(row=3, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(tracking_box, textvariable=self.pose_model_path_var).grid(row=3, column=1, sticky="ew", pady=(10, 0))

        ttk.Label(tracking_box, text="Model quality").grid(row=4, column=0, sticky="w", pady=(10, 0))
        model_quality_row = ttk.Frame(tracking_box)
        model_quality_row.grid(row=4, column=1, sticky="ew", pady=(10, 0))
        ttk.Combobox(
            model_quality_row,
            state="readonly",
            values=["lite", "full", "heavy"],
            textvariable=self.pose_model_variant_var,
            width=10,
        ).pack(side=tk.LEFT)
        ttk.Button(model_quality_row, text="Prepare Model", command=self._prepare_selected_pose_model).pack(
            side=tk.LEFT, padx=(8, 0)
        )

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

        bridge_flags = ttk.Frame(output_box)
        bridge_flags.grid(row=5, column=0, columnspan=2, sticky="w", pady=(10, 0))
        ttk.Checkbutton(bridge_flags, text="Enable HMD", variable=self.bridge_enable_hmd_var).pack(side=tk.LEFT)
        ttk.Checkbutton(
            bridge_flags,
            text="Enable Controllers",
            variable=self.bridge_enable_controllers_var,
        ).pack(side=tk.LEFT, padx=(10, 0))

        ttk.Label(output_box, text="Bridge axis X / Y / Z").grid(row=6, column=0, sticky="w", pady=(10, 0))
        axis_frame = ttk.Frame(output_box)
        axis_frame.grid(row=6, column=1, sticky="w", pady=(10, 0))
        ttk.Entry(axis_frame, textvariable=self.bridge_x_axis_var, width=7).pack(side=tk.LEFT)
        ttk.Entry(axis_frame, textvariable=self.bridge_y_axis_var, width=7).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Entry(axis_frame, textvariable=self.bridge_z_axis_var, width=7).pack(side=tk.LEFT, padx=(6, 0))

        ttk.Label(output_box, text="HMD Y offset").grid(row=7, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(output_box, textvariable=self.hmd_y_offset_var, width=10).grid(row=7, column=1, sticky="w", pady=(10, 0))
        ttk.Label(output_box, text="Hand move scale").grid(row=8, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(output_box, textvariable=self.controller_position_scale_var, width=10).grid(
            row=8, column=1, sticky="w", pady=(10, 0)
        )

        steamvr_box = ttk.LabelFrame(controls, text="SteamVR Helper", padding=14)
        steamvr_box.grid(row=2, column=0, sticky="ew", pady=(14, 0))
        steamvr_box.columnconfigure(0, weight=1)
        steamvr_box.columnconfigure(1, weight=1)
        ttk.Button(steamvr_box, text="Auto Setup SteamVR", command=self._auto_setup_steamvr).grid(
            row=0, column=0, columnspan=2, sticky="ew"
        )
        ttk.Button(steamvr_box, text="Launch VMT", command=self._launch_vmt).grid(
            row=1, column=0, sticky="ew", padx=(0, 6), pady=(10, 0)
        )
        ttk.Button(steamvr_box, text="Launch Bridge", command=self._launch_bridge).grid(
            row=1, column=1, sticky="ew", padx=(6, 0), pady=(10, 0)
        )
        ttk.Button(steamvr_box, text="Headset-Free Preset", command=self._apply_headset_free_preset).grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0)
        )
        ttk.Label(steamvr_box, textvariable=self.steamvr_var, wraplength=320, justify="left").grid(
            row=3, column=0, columnspan=2, sticky="nw", pady=(10, 0)
        )
        ttk.Label(
            steamvr_box,
            text=(
                "VMT checklist:\n"
                "1. Room Setup -> Set Room Matrix -> 0: OK\n"
                "2. Check VMT_0 Position\n"
                "3. Enable compatible controller mode if VRChat ignores hands\n"
                "4. Restart SteamVR after changing VMT setup"
            ),
            wraplength=320,
            justify="left",
        ).grid(row=4, column=0, columnspan=2, sticky="nw", pady=(10, 0))

        camera_box = ttk.LabelFrame(controls, text="Cameras", padding=14)
        camera_box.grid(row=3, column=0, sticky="ew", pady=(14, 0))
        camera_box.columnconfigure(0, weight=1)
        self.camera_rows_container = ttk.Frame(camera_box)
        self.camera_rows_container.grid(row=0, column=0, sticky="ew")

        actions = ttk.Frame(camera_box)
        actions.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        actions.columnconfigure(0, weight=1)
        actions.columnconfigure(1, weight=1)
        ttk.Button(actions, text="Rescan Cameras", command=self._rescan_cameras).grid(row=0, column=0, sticky="ew")
        ttk.Button(actions, text="Calibration Position Setup", command=self._open_camera_calibration).grid(
            row=0, column=1, sticky="ew", padx=(8, 0)
        )

        control_buttons = ttk.Frame(controls)
        control_buttons.grid(row=4, column=0, sticky="ew", pady=(14, 0))
        control_buttons.columnconfigure(0, weight=1)
        control_buttons.columnconfigure(1, weight=1)
        ttk.Button(control_buttons, text="Start Tracking", command=self._start_tracking).grid(
            row=0, column=0, sticky="ew", padx=(0, 6)
        )
        ttk.Button(control_buttons, text="Stop Tracking", command=self._stop_tracking).grid(
            row=0, column=1, sticky="ew", padx=(6, 0)
        )

        status_box = ttk.LabelFrame(controls, text="Status", padding=14)
        status_box.grid(row=5, column=0, sticky="nsew", pady=(14, 0))
        status_box.columnconfigure(0, weight=1)
        controls.rowconfigure(5, weight=1)
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

    def _on_controls_configure(self, _event) -> None:
        self.controls_canvas.configure(scrollregion=self.controls_canvas.bbox("all"))

    def _on_controls_canvas_configure(self, event) -> None:
        self.controls_canvas.itemconfigure(self.controls_window_id, width=event.width)

    def _on_mousewheel(self, event) -> None:
        if not hasattr(self, "controls_canvas"):
            return
        pointer_widget = self.winfo_containing(event.x_root, event.y_root)
        if pointer_widget is None:
            return
        parent = pointer_widget
        while parent is not None:
            if parent == self.controls_canvas:
                self.controls_canvas.yview_scroll(int(-event.delta / 120), "units")
                return
            parent = parent.master

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

    def _open_camera_calibration(self) -> None:
        if not self.camera_rows:
            messagebox.showwarning("No cameras", "Scan or add at least one camera first.")
            return
        CameraCalibrationDialog(self, self.camera_rows)

    def _set_steamvr_status(self, message: str) -> None:
        self.steamvr_var.set(message)

    def _selected_pose_model_path(self) -> str:
        variant = self.pose_model_variant_var.get().strip().lower() or "full"
        return f"models/pose_landmarker_{variant}.task"

    def _run_background_action(self, initial_message: str, action) -> None:
        self._set_steamvr_status(initial_message)

        def worker() -> None:
            try:
                action()
            except Exception as error:
                self.after(0, lambda: self._set_steamvr_status(f"SteamVR helper failed: {error}"))

        threading.Thread(target=worker, daemon=True).start()

    def _apply_headset_free_preset(self) -> None:
        self.output_enabled_var.set(True)
        self.protocol_var.set("json_udp")
        self.host_var.set("127.0.0.1")
        self.port_var.set("7000")
        self.bridge_enable_hmd_var.set(True)
        self.bridge_enable_controllers_var.set(False)
        self.bridge_x_axis_var.set(1.0)
        self.bridge_y_axis_var.set(1.0)
        self.bridge_z_axis_var.set(-1.0)
        self.hmd_y_offset_var.set(0.0)
        self.controller_position_scale_var.set(2.2)
        self._set_steamvr_status("Headset-free preset applied: json_udp -> 127.0.0.1:7000")

    def _prepare_selected_pose_model(self) -> None:
        self.pose_model_path_var.set(self._selected_pose_model_path())

        def action() -> None:
            ensure_pose_model(
                self._threadsafe_steamvr_message,
                self.pose_model_path_var.get().strip(),
                variant=self.pose_model_variant_var.get().strip(),
            )
            variant = self.pose_model_variant_var.get().strip().lower() or "full"
            self.after(0, lambda: self.status_var.set(f"Pose model prepared: {variant}"))

        self._run_background_action("Preparing pose model...", action)

    def _auto_setup_steamvr(self) -> None:
        self._apply_headset_free_preset()

        def action() -> None:
            self.pose_model_path_var.set(self._selected_pose_model_path())
            ensure_pose_model(
                self._threadsafe_steamvr_message,
                self.pose_model_path_var.get().strip(),
                variant=self.pose_model_variant_var.get().strip(),
            )
            launch_vmt_manager(self._threadsafe_steamvr_message)
            launch_bridge(
                self._threadsafe_steamvr_message,
                listen_port=7000,
                hmd_port=39575,
                enable_hmd=self.bridge_enable_hmd_var.get(),
                enable_controllers=self.bridge_enable_controllers_var.get(),
                x_axis_scale=float(self.bridge_x_axis_var.get()),
                y_axis_scale=float(self.bridge_y_axis_var.get()),
                z_axis_scale=float(self.bridge_z_axis_var.get()),
                hmd_y_offset=float(self.hmd_y_offset_var.get()),
                controller_position_scale=float(self.controller_position_scale_var.get()),
            )
            self.after(0, lambda: self._set_steamvr_status("SteamVR helper ready. VMT and bridge are running."))

        self._run_background_action("Preparing SteamVR helper...", action)

    def _launch_vmt(self) -> None:
        def action() -> None:
            launch_vmt_manager(self._threadsafe_steamvr_message)
            self.after(0, lambda: self._set_steamvr_status("VMT Manager is ready."))

        self._run_background_action("Launching VMT Manager...", action)

    def _launch_bridge(self) -> None:
        def action() -> None:
            launch_bridge(
                self._threadsafe_steamvr_message,
                listen_port=7000,
                hmd_port=39575,
                enable_hmd=self.bridge_enable_hmd_var.get(),
                enable_controllers=self.bridge_enable_controllers_var.get(),
                x_axis_scale=float(self.bridge_x_axis_var.get()),
                y_axis_scale=float(self.bridge_y_axis_var.get()),
                z_axis_scale=float(self.bridge_z_axis_var.get()),
                hmd_y_offset=float(self.hmd_y_offset_var.get()),
                controller_position_scale=float(self.controller_position_scale_var.get()),
            )
            self.after(0, lambda: self._set_steamvr_status("Bridge is listening on UDP 7000."))

        self._run_background_action("Launching bridge...", action)

    def _threadsafe_steamvr_message(self, message: str) -> None:
        self.after(0, lambda: self._set_steamvr_status(message))

    def _collect_config(self) -> AppConfig:
        tracking = TrackingConfig(
            mode=TrackingMode(self.mode_var.get()),
            min_visibility=float(self.min_visibility_var.get()),
            smoothing=float(self.smoothing_var.get()),
            output_fps=self.config_model.tracking.output_fps,
            model_complexity=self.config_model.tracking.model_complexity,
            max_cameras_to_scan=self.config_model.tracking.max_cameras_to_scan,
            pose_model_path=self.pose_model_path_var.get().strip() or self._selected_pose_model_path(),
        )
        output = OutputConfig(
            enabled=self.output_enabled_var.get(),
            protocol=self.protocol_var.get(),
            host=self.host_var.get().strip() or "127.0.0.1",
            port=int(self.port_var.get()),
            emit_joints=self.emit_joints_var.get(),
            bridge_enable_hmd=self.bridge_enable_hmd_var.get(),
            bridge_enable_controllers=self.bridge_enable_controllers_var.get(),
            bridge_x_axis_scale=float(self.bridge_x_axis_var.get()),
            bridge_y_axis_scale=float(self.bridge_y_axis_var.get()),
            bridge_z_axis_scale=float(self.bridge_z_axis_var.get()),
            hmd_y_offset=float(self.hmd_y_offset_var.get()),
            controller_position_scale=float(self.controller_position_scale_var.get()),
        )
        cameras = [row.as_config() for row in self.camera_rows]
        return AppConfig(tracking=tracking, output=output, cameras=cameras)

    def _start_tracking(self) -> None:
        try:
            config = self._collect_config()
        except ValueError:
            messagebox.showerror("Invalid configuration", "Port and numeric settings must be valid numbers.")
            return

        missing = detect_missing_runtime_dependencies(config)
        if missing:
            help_text = (
                "Tracking runtime is not ready.\n\n"
                "For MediaPipe Tasks on Python 3.14, put a pose landmarker model at the configured path above.\n"
                "Default: models/pose_landmarker_full.task\n\n"
                "If you want to keep using the older MediaPipe Solutions backend instead, use Python 3.12 x64.\n\n"
                "Details: "
            )
            messagebox.showerror(
                "Tracking runtime not ready",
                help_text + ", ".join(missing),
            )
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
        self._refresh_steamvr_status()

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

    def _refresh_steamvr_status(self) -> None:
        status = get_steamvr_status()
        manager_state = "running" if status.vmt_running else ("installed" if status.vmt_manager_path else "missing")
        bridge_state = "running" if status.bridge_port_open else "stopped"
        if status.hmd_port_open:
            hmd_state = "running"
        elif status.hmd_driver_registered:
            hmd_state = "registered, not active"
        elif status.hmd_driver_built:
            hmd_state = "built, not registered"
        else:
            hmd_state = "missing build"
        self._set_steamvr_status(f"VMT: {manager_state} | Bridge: {bridge_state} | HMD: {hmd_state}")

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
