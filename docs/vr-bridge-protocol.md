# VR Bridge Protocol

The tracker can emit either OSC messages or JSON/UDP datagrams.

## Tracker names

The app currently derives these tracker points when the required landmarks are available:

- `head`
- `neck`
- `chest`
- `waist`
- `left_hand`
- `right_hand`
- `left_elbow`
- `right_elbow`
- `left_knee`
- `right_knee`
- `left_foot`
- `right_foot`

Upper-body mode omits knee and foot trackers.

## OSC payloads

### `/tracking/meta`

Arguments:

- `timestamp` as float
- `active_cameras` as int
- `tracker_count` as int

### `/tracking/tracker/<name>`

Arguments:

- `x` as float
- `y` as float
- `z` as float
- `confidence` as float

### `/tracking/joint/<name>`

Arguments:

- `x` as float
- `y` as float
- `z` as float
- `visibility` as float

## JSON/UDP packet

```json
{
  "timestamp": 1713000000.0,
  "mode": "full_body",
  "active_cameras": [0, 1],
  "joints": {
    "left_shoulder": {
      "x": -0.14,
      "y": 0.31,
      "z": -0.08,
      "visibility": 0.98
    }
  },
  "trackers": {
    "waist": {
      "x": 0.01,
      "y": -0.05,
      "z": 0.02,
      "confidence": 0.96
    }
  }
}
```

## Coordinate space

- units are MediaPipe-style world coordinates
- coordinates are person-relative and best treated as a live motion space
- `waist` is a good anchor for downstream re-centering

## Recommended bridge pattern

1. receive OSC or JSON/UDP
2. re-center around `waist`
3. scale to your target runtime if needed
4. map tracker names to your VR runtime's virtual devices
5. apply final IK or smoothing in the bridge layer

## Included SteamVR bridge

This repository includes `tools/vmt_bridge.py`, which receives `json_udp` packets and forwards them to Virtual Motion Tracker (VMT) for SteamVR-compatible virtual trackers.
