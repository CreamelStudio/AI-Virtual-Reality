# VMT Bridge

This bridge receives `json_udp` packets from the tracker GUI and forwards them to Virtual Motion Tracker (VMT), which can expose the data to SteamVR as virtual trackers.

If your tracker app can already reach the Windows machine running VMT directly, the simpler option is to choose `vmt_osc` in the GUI and send straight to VMT port `39570`. The extra bridge is still recommended when you want headset-free mode or hand-gesture controller emulation.

For headset-free mode, this bridge can also forward `head` pose to the custom `aitrackinghmd` OpenVR driver on UDP port `39575`.
It also converts hand pose + finger landmarks into two VMT compatible controllers by default.

## Recommended flow

1. Install and launch SteamVR on Windows.
2. Install and launch VMT.
3. Start this bridge script on the same Windows machine as VMT.
4. Start the tracking GUI and set output to `json_udp`.
5. Point the tracker app to the bridge host and port.

## Start the bridge

```bash
python tools/vmt_bridge.py --listen-port 7000 --hmd-port 39575
```

Optional tuning:

```bash
python tools/vmt_bridge.py \
  --listen-port 7000 \
  --hmd-port 39575 \
  --scale 1.0 \
  --waist-height 0.95 \
  --x-axis-scale 1.0 \
  --y-axis-scale 1.0 \
  --z-axis-scale 1.0
```

Disable gesture controllers if a game only needs trackers:

```bash
python tools/vmt_bridge.py --listen-port 7000 --disable-controllers
```

## Tracker app settings

- protocol: `json_udp`
- host: the Windows machine running the bridge
- port: the bridge listen port, default `7000`

## Default VMT tracker map

- `waist` -> VMT index `0`
- `chest` -> VMT index `1`
- `head` -> VMT index `2`
- `left_hand` -> VMT index `3`
- `right_hand` -> VMT index `4`
- `left_foot` -> VMT index `5`
- `right_foot` -> VMT index `6`
- `left_knee` -> VMT index `7`
- `right_knee` -> VMT index `8`
- `left_elbow` -> VMT index `9`
- `right_elbow` -> VMT index `10`

All devices are sent in VMT's tracker mode `7`, which is the VIVE Tracker compatible mode.

## Default controller map

- `left_controller` -> VMT index `20`, controller mode `5`
- `right_controller` -> VMT index `21`, controller mode `6`
- closed hand (`index` + `pinky` span) -> grip

For legacy-input games, VMT's compatible controller mode is usually the safer choice than finger-only input.

## Calibration behavior

On the first valid frame the bridge will:

- use `waist` as the room origin
- align your current facing direction to room forward
- place the floor under your detected feet if available
- fall back to the configured waist height for upper-body mode

## When to tweak axis scales

If motion looks mirrored or upside down in SteamVR, try:

- `--x-axis-scale -1`
- `--y-axis-scale -1`
- `--z-axis-scale -1`

Only flip one axis at a time and test again.

## Important limitation

This bridge creates virtual trackers and basic gesture-driven controllers, but some VR games still need a full headset device.

Use `docs/headset-free-steamvr.md` if you also want the custom virtual HMD driver.
