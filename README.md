# lerobot_teleoperator_omy

A [LeRobot](https://github.com/huggingface/lerobot) teleoperator plugin for the
**ROBOTIS OMY-L100**, the 6-DoF leader arm from the
[OMY (OpenMANIPULATOR-Y)](https://ai.robotis.com/omy/introduction_omy.html)
imitation-learning platform. Drop-in `--teleop.type=omy_leader` for
`lerobot-teleoperate` and `lerobot-record`.

It talks to the leader's Dynamixel motors directly over Protocol 2.0
`GroupSyncRead` — **no ROS 2 stack required** — and returns each joint's native
angle in radians, so you can pair it with any follower (an OMY follower, or a
different arm via a small mapping processor).

> Uses LeRobot's [plugin system](https://huggingface.co/docs/lerobot), so it
> installs alongside LeRobot without patching core.

## Why this plugin

LeRobot core ships the **5-DoF OMX** (`omx_leader` / `omx_follower`,
OpenMANIPULATOR-X), but there is **no OMY** in core — and OMY is the 6-DoF arm
ROBOTIS designed specifically for imitation learning. The only official OMY
software path is the ROS 2 stack
([`open_manipulator`](https://github.com/ROBOTIS-GIT/open_manipulator) +
[`physical_ai_tools`](https://github.com/ROBOTIS-GIT/physical_ai_tools)), which
is heavyweight if all you want is to use the OMY-L100 as a leader inside a plain
LeRobot pipeline.

| | This plugin | LeRobot core `omx_leader` | ROBOTIS `physical_ai_tools` |
|---|---|---|---|
| Arm | **OMY-L100 (6-DoF)** | OMX (5-DoF) | OMY (6-DoF) |
| ROS 2 required | ❌ no | ❌ no | ✅ yes |
| Install | `pip install -e .` | bundled | ROS 2 workspace build |
| Output | native joint radians | normalized joints | ROS 2 topics |
| Gripper spring effect | ✅ (current-control restoring torque) | ✅ | ✅ |
| Robust sync-read (retry + last-good fallback) | ✅ | n/a | n/a |

To the best of our knowledge this is the first standalone, pip-installable
LeRobot teleoperator for the OMY-L100 that does not require ROS 2.

## Hardware

The OMY-L100 leader has **7 X-series Dynamixel motors** (verified by bus scan):

| Joint | Motor ID | Model |
|---|---|---|
| `joint_1`–`joint_3` (base/shoulder/elbow) | 1, 2, 3 | XH540-W150 |
| `joint_4`–`joint_6` (wrist) | 4, 5, 6 | XC330-T288 |
| `gripper` | 7 | XC330-T181 |

Default bus settings: `4 Mbps`, Protocol 2.0.

## Install

```bash
# In your LeRobot environment
pip install -e .
```

This pulls in `dynamixel-sdk`. Make sure your user can access the serial port
(e.g. add yourself to the `dialout` group, or set a udev rule).

## Usage

The plugin self-registers via
`@TeleoperatorConfig.register_subclass("omy_leader")`, so after install the
teleop type is available on every LeRobot CLI:

```bash
lerobot-teleoperate \
    --teleop.type=omy_leader \
    --teleop.port=/dev/ttyUSB0 \
    --robot.type=...          # your follower
```

### Output

`get_action()` returns the seven native joint angles, in **radians**, keyed by:

```
joint_1.pos joint_2.pos joint_3.pos joint_4.pos joint_5.pos joint_6.pos gripper.pos
```

Each value is the motor's Present_Position re-centered on the X-series zero
(`2048` raw units) and converted to radians. Mapping these onto a *different*
follower's joint space (scale / offset / limits) is intentionally **not** done
here — keep that in a downstream LeRobot processor so this teleoperator stays a
generic OMY-L100 reader.

### Key config options

| Option | Default | Meaning |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Serial port of the leader |
| `baudrate` | `4_000_000` | Dynamixel bus baud rate |
| `motor_ids` | `[1,2,3,4,5,6,7]` | Motor IDs in joint order (arm + gripper) |
| `smoothing_factor` | `0.0` | EMA smoothing on readings (0 = off) |
| `gripper_spring_enabled` | `True` | Apply restoring torque so the trigger springs open |
| `gripper_spring_stiffness` / `_damping` / `_max_current` | `0.06` / `0.004` / `150` | Spring tuning + safety clamp |

See [`config_omy_leader.py`](src/lerobot_teleoperator_omy/config_omy_leader.py)
for the full list.

## Package layout

```
src/lerobot_teleoperator_omy/
├── omy_leader.py          # OmyLeader teleoperator
└── config_omy_leader.py   # OmyLeaderConfig (registered as "omy_leader")
```

## License

[Apache-2.0](LICENSE).

This is an independent, community-maintained plugin and is not affiliated with
ROBOTIS or Hugging Face. "OMY", "OpenMANIPULATOR", and "ROBOTIS" are trademarks
of their respective owners.
