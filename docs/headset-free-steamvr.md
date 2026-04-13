# Headset-Free SteamVR Mode

This mode combines:

- AI tracker app
- Windows-side `vmt_bridge.py`
- VMT for virtual body trackers
- VMT compatible controllers driven by hand gestures
- custom OpenVR HMD driver for a virtual headset

## What it does

The bridge receives one `json_udp` packet stream from the tracker app.

From that single stream it:

- forwards body tracker poses to VMT
- converts both hands into virtual controllers
- forwards head pose to the custom `aitrackinghmd` OpenVR driver

That gives SteamVR both:

- a virtual HMD
- full-body virtual trackers
- basic controller input without physical VR controllers

## Default hand gesture controls

- left hand -> VMT controller index `20`
- right hand -> VMT controller index `21`
- pinch (`thumb` to `index`) -> trigger
- close hand (`index` to `pinky` span shrinks) -> grip
- move wrist relative to shoulder -> joystick
- raised pinch -> `A`
- raised grip -> `B`

If the hand landmarks are noisy for a specific game, start the bridge with `--disable-controllers` and use keyboard or gamepad remapping instead.

## Important limitation

This gives SteamVR an HMD, trackers, and basic controller input, but some games still expect richer controller behavior than body-pose hand gestures can provide.

## Official references used

- Valve OpenVR driver documentation:
  https://github.com/ValveSoftware/openvr/wiki/Driver-Documentation
- Valve local driver registration:
  https://github.com/ValveSoftware/openvr/wiki/Local-Driver-Registration
- Valve sample HMD driver:
  https://github.com/ValveSoftware/openvr/tree/master/samples/drivers/drivers/simplehmd

## Build the HMD driver on Windows

1. Clone or copy the OpenVR SDK locally.
2. Place this project folder where you can build it with Visual Studio 2019 or 2022.
3. Add `steamvr/ai_headless_hmd_driver` into the OpenVR sample solution or build it with CMake using the same OpenVR sample environment.
4. Build the `driver_aitrackinghmd` target for `x64`.

Expected output folder:

```text
build/output/aitrackinghmd
```

## Register the driver

Run on Windows PowerShell:

```powershell
cd steamvr/ai_headless_hmd_driver
./install_driver.ps1
```

That uses `vrpathreg.exe adddriver` to register the built driver with SteamVR.

## Start the full pipeline

1. Start SteamVR.
2. Start VMT.
3. Make sure the custom HMD driver is registered.
4. Start the bridge on the Windows machine:

```bash
python tools/vmt_bridge.py --listen-port 7000 --hmd-port 39575
```

5. Start the tracker GUI and set:

- protocol: `json_udp`
- host: the Windows machine running SteamVR
- port: `7000`

## Pose protocol between bridge and HMD driver

The bridge sends one UDP text packet to the HMD driver for each frame:

```text
x y z qx qy qz qw connected
```

Example:

```text
0.000 1.620 0.050 0.000 0.000 0.000 1.000 1
```

## Tuning notes

- `aitrackinghmd/resources/settings/default.vrsettings` controls the HMD UDP port and desktop render size.
- `tools/vmt_bridge.py` can be pointed to another HMD UDP port with `--hmd-port`.
- use `--disable-controllers` if you want headset-free mode without gesture controllers
- If head direction looks wrong, adjust the body calibration first before changing the driver.
