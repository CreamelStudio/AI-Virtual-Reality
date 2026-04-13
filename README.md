# AI Body Tracking VR Bridge

Desktop GUI prototype for:

- full-body tracking
- upper-body-only tracking
- multi-webcam fusion for better robustness
- VR bridge style signal output over OSC or JSON/UDP

The app uses MediaPipe Pose for human tracking, combines multiple camera observations into one fused skeleton, and publishes tracker data that can be consumed by a VR bridge or a custom runtime.

## What is included

- Tkinter GUI for camera selection and runtime controls
- multi-camera capture and weighted landmark fusion
- full-body and upper-body modes
- fused skeleton preview
- OSC output for live tracker streaming
- JSON/UDP output for debugging or custom bridge apps
- mock runtime listener for quick local verification

## Project layout

- `run.py`: launch entrypoint
- `src/ai_vr_tracking/`: application source
- `tools/mock_vr_runtime.py`: simple UDP listener for testing output
- `docs/vr-bridge-protocol.md`: emitted signal format

## Quick start

1. Create a virtual environment.
2. Install dependencies.
3. Start the GUI.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 run.py
```

On Windows, both of these setups are supported:

- Python 3.12 with the classic `mediapipe.solutions.pose` backend
- Python 3.14 with the newer MediaPipe Tasks backend and a local model file at `models/pose_landmarker_full.task`

If you use Python 3.14, download the Pose Landmarker task model and keep it at the configured path shown in the GUI.

## Usage flow

1. Press `Rescan Cameras`.
2. Enable one or more webcams.
3. Pick `full_body` or `upper_body`.
4. Set smoothing and minimum visibility.
5. Choose `osc` or `json_udp`.
6. Press `Start Tracking`.

## Output modes

### OSC

Designed for a future VR bridge or external router. The app emits:

- `/tracking/meta`
- `/tracking/tracker/<name>`
- `/tracking/joint/<name>`

See `docs/vr-bridge-protocol.md` for details.

### JSON/UDP

Sends one JSON packet per frame containing timestamp, mode, fused joints, and tracker positions. This is the easiest mode for debugging and integration work.

Run the mock listener in another terminal:

```bash
python3 tools/mock_vr_runtime.py --port 7000
```

Then set the GUI output protocol to `json_udp` and the port to `7000`.

### VMT direct

If VMT is already running on the target Windows machine, you can skip the extra relay and send poses straight from the GUI:

- protocol: `vmt_osc`
- host: the machine running VMT
- port: `39570`

## SteamVR bridge

To forward the tracking data into SteamVR virtual trackers, run the Windows-side VMT bridge:

```bash
python tools/vmt_bridge.py --listen-port 7000 --hmd-port 39575
```

Then in the GUI:

- protocol: `json_udp`
- host: the machine running the bridge
- port: `7000`

See `docs/vmt-bridge.md` for the full setup and tuning options. If you do not need the extra relay step, use the GUI's `vmt_osc` output mode instead.
The bridge path is the one that also adds gesture-driven compatible controllers.

## Headset-Free SteamVR

For games that refuse to start without an HMD, use the custom virtual headset flow described in `docs/headset-free-steamvr.md`. That mode combines:

- the tracker app
- the Windows-side bridge
- VMT virtual trackers
- gesture-driven VMT compatible controllers
- the `aitrackinghmd` OpenVR driver

## Notes on VR game integration

This repository now includes a Windows-side VMT bridge for SteamVR virtual trackers, gesture-driven controllers, and a custom virtual HMD path for headset-free launch on Windows.

## Suggested next steps

- add per-camera calibration profiles
- add recording and replay
