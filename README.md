# LeRobot + ROBOTIS OMY-L100 Integration

Brings a [LeRobot](https://github.com/huggingface/lerobot) teleoperator
integration for the **ROBOTIS OMY-L100**, the 6-DoF leader arm from the
[OMY (OpenMANIPULATOR-Y)](https://ai.robotis.com/omy/introduction_omy.html)
imitation-learning platform. Registers `omy_leader`, so it works with
`lerobot-teleoperate` and `lerobot-record` without patching LeRobot.

It reads the leader's X-series Dynamixel motors directly over Protocol 2.0
`GroupSyncRead` — **no ROS 2 stack** — and returns each joint's native angle in
radians, so you can pair it with any follower (an OMY follower, or another arm
via a small mapping processor).

## Getting Started

```bash
# In your LeRobot environment
pip install git+https://github.com/charlie8612/lerobot_teleoperator_omy.git

lerobot-teleoperate \
    --teleop.type=omy_leader \
    --teleop.port=/dev/ttyUSB0 \
    --robot.type=...          # your follower
```

The plugin self-registers via `@TeleoperatorConfig.register_subclass("omy_leader")`,
so the teleop type is available on every LeRobot CLI after install. Make sure
your user can access the serial port (e.g. add yourself to the `dialout` group).

## Why this plugin

LeRobot core ships the **5-DoF OMX** (`omx_leader`), but there is **no OMY** in
core — and OMY is the 6-DoF arm ROBOTIS designed for imitation learning. The
only official OMY software path is the heavyweight ROS 2 stack.

| | This plugin | core `omx_leader` | ROBOTIS `physical_ai_tools` |
|---|---|---|---|
| Arm | **OMY-L100 (6-DoF)** | OMX (5-DoF) | OMY (6-DoF) |
| ROS 2 required | ❌ | ❌ | ✅ |
| Install | `pip install` | bundled | ROS 2 workspace |
| Output | native joint radians | normalized joints | ROS 2 topics |
| Gripper spring effect | ✅ | ✅ | ✅ |
| Robust sync-read (retry + last-good fallback) | ✅ | n/a | n/a |

To our knowledge this is the first standalone, pip-installable LeRobot
teleoperator for the OMY-L100 that does not require ROS 2.

## Output

`get_action()` returns the seven native joint angles in **radians**:

```
joint_1.pos joint_2.pos joint_3.pos joint_4.pos joint_5.pos joint_6.pos gripper.pos
```

Each value is the motor's Present_Position re-centered on the X-series zero and
converted to radians. Mapping these onto a *different* follower's joint space is
intentionally left to a downstream LeRobot processor, so this teleoperator stays
a generic OMY-L100 reader.

## Hardware

7 X-series Dynamixel motors (verified by bus scan), 4 Mbps, Protocol 2.0:

| Joint | Motor ID | Model |
|---|---|---|
| `joint_1`–`joint_3` (base/shoulder/elbow) | 1–3 | XH540-W150 |
| `joint_4`–`joint_6` (wrist) | 4–6 | XC330-T288 |
| `gripper` | 7 | XC330-T181 |

## Configuration

| Option | Default | Meaning |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Serial port of the leader |
| `baudrate` | `4_000_000` | Dynamixel bus baud rate |
| `motor_ids` | `[1,2,3,4,5,6,7]` | Motor IDs in joint order (arm + gripper) |
| `smoothing_factor` | `0.0` | EMA smoothing on readings (0 = off) |
| `gripper_spring_enabled` | `True` | Restoring torque so the trigger springs open |

See [`config_omy_leader.py`](src/lerobot_teleoperator_omy/config_omy_leader.py)
for the full list.

## Development

```bash
git clone https://github.com/charlie8612/lerobot_teleoperator_omy.git
cd lerobot_teleoperator_omy
pip install -e ".[dev]"
pytest -q
```

## License

[Apache-2.0](LICENSE). Independent, community-maintained plugin, not affiliated
with ROBOTIS or Hugging Face. "OMY", "OpenMANIPULATOR", and "ROBOTIS" are
trademarks of their respective owners.
