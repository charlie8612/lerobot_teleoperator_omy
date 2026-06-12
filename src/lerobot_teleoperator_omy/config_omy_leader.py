from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig


@dataclass
class OmyLeaderBaseConfig:
    """Base configuration for the ROBOTIS OMY-L100 leader arm (not registered with draccus).

    Reads joint positions directly from the OMY-L100's X-series Dynamixel motors
    (XH540-W150 x3, XC330-T288 x3, XC330-T181 gripper) over Protocol 2.0
    ``GroupSyncRead`` — no ROS 2 stack required. The teleoperator outputs each
    joint's *native* angle in radians; mapping onto a particular follower (e.g. an
    OMY follower or a different arm) is left to a downstream processor.
    """

    # Serial port the leader is connected to (e.g. "/dev/ttyUSB0").
    port: str = "/dev/ttyUSB0"

    # Dynamixel bus baud rate. OMY-L100 ships at 4 Mbps.
    baudrate: int = 4_000_000

    # Dynamixel protocol version (X-series = 2.0).
    protocol_version: float = 2.0

    # Motor IDs in joint order: 6 arm joints (1-6) + gripper (7).
    motor_ids: list[int] = field(default_factory=lambda: [1, 2, 3, 4, 5, 6, 7])

    # Index into motor_ids of the gripper motor (last entry by default).
    gripper_index: int = 6

    # X-series position resolution: 4096 raw units = 1 revolution (2*pi rad).
    units_per_revolution: int = 4096

    # X-series center position (raw units) treated as 0 rad.
    position_zero_offset: int = 2048

    # Operating mode written to the arm joints on connect. 4 = Extended Position
    # (multi-turn) so a joint can rotate past 360 deg without Present_Position
    # wrapping at 0/4095. The gripper is always put in Current mode for the spring.
    arm_operating_mode: int = 4

    # Exponential moving-average smoothing on the joint readings.
    # 0.0 = no smoothing, 0.9 = very smooth (more lag).
    smoothing_factor: float = 0.0

    # --- Gripper spring effect -------------------------------------------------
    # The gripper motor runs in Current Control mode so it can apply a restoring
    # torque, letting the operator squeeze the trigger and have it spring back to
    # an open position on its own (mirrors the ROBOTIS ROS 2 spring controller).
    gripper_spring_enabled: bool = True
    gripper_spring_stiffness: float = 0.06   # N*m/rad
    gripper_spring_neutral_rad: float = 0.0  # open position
    gripper_spring_damping: float = 0.004    # N*m*s/rad
    gripper_spring_max_current: int = 150    # Goal Current units (safety clamp)


@TeleoperatorConfig.register_subclass("omy_leader")
@dataclass
class OmyLeaderConfig(TeleoperatorConfig, OmyLeaderBaseConfig):
    """Configuration for the ROBOTIS OMY-L100 leader arm teleoperator."""
    pass
