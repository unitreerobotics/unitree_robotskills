# G1-D connection examples

English | [中文](README.zh.md)

Run the examples from `unitree_gpt6/g1d_draw` after installing its dependencies. The first three examples read feedback or images without taking arm control or requiring a URDF.

| File | Purpose |
| --- | --- |
| [check_connection.py](check_connection.py) | Check the network interface, DDS feedback, camera configuration and decoded image streams |
| [read_joints.py](read_joints.py) | Display and save current arm, waist and Dex1 feedback |
| [capture_images.py](capture_images.py) | Subscribe to JPEG streams and save PNG images with provenance |
| [execute.py](execute.py) | Execute a compiled, reviewed job; this example moves the robot |

## Install and configure

```bash
# From the repository root / 在仓库根目录执行
python -m pip install -r unitree_gpt6/g1d_draw/requirements.txt
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
cd unitree_gpt6/g1d_draw
```

Use a fresh Python 3.10 virtual environment. [requirements.txt](../requirements.txt) pins dependency versions and the SDK revision; image processing uses `opencv-python` and PyZMQ. If CycloneDDS installation fails, follow the [SDK installation instructions](https://github.com/unitreerobotics/unitree_sdk2_python#installation) and set `CYCLONEDDS_HOME`.

Copy and edit [device.example.json](../config/device.example.json), or override the wired DDS interface as shown below. Joint observation does not use the model or gain fields.

```bash
ip -br addr
IFACE=enp4s0
CAMERA_HOST=192.168.123.164
ip route get "$CAMERA_HOST"
```

Replace the example interface and camera address with your deployment values. DDS binds to `--interface`; camera TCP connections use the system route. Check the `dev` reported by `ip route get`. The camera server may run on the robot or a separate image host, with a different address from the DDS device.

## 1. Check connectivity

```bash
python examples/check_connection.py \
  --config config/device.example.json --interface "$IFACE" \
  --camera-host "$CAMERA_HOST" --timeout 5 \
  --output ./captures/connection.json
```

This reads fresh joint feedback, requests the camera configuration, then receives and decodes a JPEG from each enabled stream. Exit code `0` means all requested checks passed; `1` means at least one failed. The JSON records individual results. Supply only `--config` to check DDS, or only `--camera-host` for cameras. Set `--camera-config-port` if the configuration service uses another port.

When only a stream endpoint is known, test it directly:

```bash
python examples/check_connection.py \
  --stream "head=tcp://$CAMERA_HOST:55555" \
  --output ./captures/head-connection.json
```

## 2. Read and record joints

```bash
python examples/read_joints.py \
  --config config/device.example.json --interface "$IFACE" \
  --samples 3 --interval 0.5 --output ./captures/joints.json
```

The table shows names, slots, radians, degrees and velocities. JSON includes `q14`, `dq14`, waist, grippers and timing. The polling interval is not the DDS publication frequency.

| Data | Order and units |
| --- | --- |
| `q14` / `dq14` | Left arm 7, then right arm 7; rad / rad/s; LowState slots 15–28 |
| Within each arm | shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw |
| `waist_q3` | yaw, roll, pitch; rad; slots 12, 13, 14 |
| `grip_q2` | Left, right; native device units, not percentages; internal Dex1 slots 31, 33 |

External grippers require explicit left/right state and command topics in the device configuration. This script uses only `connect/read/health/close` and creates no command publisher.

## 3. Read and save camera images

The camera side must run [TeleImager](https://github.com/unitreerobotics/teleimager) or a compatible server with ZMQ enabled. The client sends `GET_DATA` to the configuration service for JSON, then subscribes to each stream's `zmq_port`, receiving one JPEG per message. This is separate from joint DDS; installing the SDK does not start a camera server.

Common ports are shown below; use the actual configuration returned by the connection check.

| Camera | Example ZMQ port | Image |
| --- | --- | --- |
| `head_camera` | 55555 | May contain horizontally joined stereo views; inspect `binocular` and dimensions |
| `left_wrist_camera` | 55556 | Left wrist image |
| `right_wrist_camera` | 55557 | Right wrist image |
| Camera configuration | 60000 | REQ/REP configuration service, not an image stream |

```bash
python examples/capture_images.py \
  --stream "head=tcp://$CAMERA_HOST:55555" \
  --stream "left_wrist=tcp://$CAMERA_HOST:55556" \
  --stream "right_wrist=tcp://$CAMERA_HOST:55557" \
  --warmup-frames 3 --timeout 10 --output ./captures/images-1
```

Outputs are `head.png`, `left_wrist.png`, `right_wrist.png` and `capture.json`, recording endpoints, dimensions, hashes and host receipt times. Omit unwanted streams. Full images are saved without cropping, mirroring or inferred orientation. Streams are not exposure-synchronized, and joint snapshots are not synchronized with images.

## Troubleshooting

| Result | Check |
| --- | --- |
| Interface not found | Use `ip -br addr`; check the cable and address configuration |
| DDS timeout | Interface, domain, LowState topic/type, robot state publisher and DDS network configuration; ping alone does not verify DDS |
| Camera configuration timeout | Image host, configuration port and running server; test known stream endpoints independently |
| Configuration works but a stream has no images | `enable_zmq`, actual port, camera connection and JPEG publisher |
| Data arrives but cannot be decoded | The example accepts single-message JPEG/ZMQ, not WebRTC, RTSP or other framing |
| Output already exists | Choose a new file or directory; existing evidence is preserved |

## Execute motion

A joint snapshot is feedback, not an executable job. For measurement motion or painting, configure the model, routes and measurements using the main [README](../README.md), then use `compile-probe/probe` or the exported `run.sh`. See [execute.py](execute.py) for the Python API.

The execution example shares the main command's controller lifecycle and retains hold after completion or stop. Ctrl-C stops an active run; after completion or stop, Ctrl-C releases. Another terminal can also use the main command's `status`, `stop`, and `release` commands.
