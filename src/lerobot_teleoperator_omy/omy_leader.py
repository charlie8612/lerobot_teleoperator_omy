import logging
import math
import struct

from dynamixel_sdk import GroupSyncRead, PacketHandler, PortHandler

from lerobot.processor import RobotAction
from lerobot.teleoperators.teleoperator import Teleoperator

from .config_omy_leader import OmyLeaderConfig

logger = logging.getLogger(__name__)

# X-series Dynamixel control table (XH540-W150, XC330-T288, XC330-T181)
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
ADDR_PRESENT_VELOCITY = 128

# XC330-T181 current unit: N*m per Goal Current unit (from the model spec).
XC330_T181_CURRENT_UNIT = 0.0006709470296015791


class OmyLeader(Teleoperator):
    """ROBOTIS OMY-L100 leader arm teleoperator.

    Reads the six arm joints plus the gripper from the OMY-L100's X-series
    Dynamixel motors via ``dynamixel_sdk`` Protocol 2.0 ``GroupSyncRead`` and
    returns each joint's native angle in radians as ``joint_<n>.pos`` /
    ``gripper.pos``. No ROS 2 stack is required.

    The gripper motor optionally runs a spring effect (Current Control mode) so
    the trigger returns to the open position on its own.
    """

    config_class = OmyLeaderConfig
    name = "omy_leader"

    def __init__(self, config: OmyLeaderConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._port_handler: PortHandler | None = None
        self._packet_handler: PacketHandler | None = None
        self._sync_reader: GroupSyncRead | None = None
        self._prev_rad: list[float] | None = None

        self._joint_names = [
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6",
            "gripper",
        ]

    # ---- Properties ----

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{name}.pos": float for name in self._joint_names}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    # ---- Lifecycle ----

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            raise RuntimeError("OmyLeader is already connected.")

        port = PortHandler(self.config.port)
        if not port.openPort():
            raise ConnectionError(f"Failed to open port {self.config.port}")
        if not port.setBaudRate(self.config.baudrate):
            port.closePort()
            raise ConnectionError(f"Failed to set baud rate {self.config.baudrate}")

        packet = PacketHandler(self.config.protocol_version)
        self._port_handler = port
        self._packet_handler = packet

        # Disable torque on all motors so the arm can be moved freely by hand.
        for mid in self.config.motor_ids:
            result, _ = packet.write1ByteTxRx(port, mid, ADDR_TORQUE_ENABLE, 0)
            if result != 0:
                logger.warning(f"Motor {mid} torque disable failed: {packet.getTxRxResult(result)}")

        # Enable torque on the gripper motor only, so it can apply the spring force.
        # The motor is in Current Control mode, so enabling torque lets us write
        # Goal Current without commanding a position.
        gripper_mid = self.config.motor_ids[self.config.gripper_index]
        if self.config.gripper_spring_enabled:
            result, _ = packet.write1ByteTxRx(port, gripper_mid, ADDR_TORQUE_ENABLE, 1)
            if result != 0:
                logger.warning(f"Gripper motor {gripper_mid} torque enable failed")
            else:
                logger.info(f"Gripper spring enabled on motor {gripper_mid}")

        # Set up the sync reader for Present_Position across all motors.
        reader = GroupSyncRead(port, packet, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        for mid in self.config.motor_ids:
            if not reader.addParam(mid):
                raise RuntimeError(f"Failed to add motor {mid} to sync reader")
        self._sync_reader = reader

        # Verify we can read before reporting connected.
        positions = self._read_positions_rad()
        self._is_connected = True
        logger.info(
            f"OmyLeader connected on {self.config.port} @ {self.config.baudrate} bps, "
            f"{len(self.config.motor_ids)} motors"
        )
        logger.info(f"  Initial (rad): {[f'{v:+.4f}' for v in positions]}")

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_action(self) -> RobotAction:
        """Read the leader's native joint angles (radians) and return them."""
        positions_rad = self._read_positions_rad()
        if self.config.gripper_spring_enabled:
            self._apply_gripper_spring(positions_rad)
        return {
            f"{name}.pos": float(positions_rad[i])
            for i, name in enumerate(self._joint_names)
        }

    def send_feedback(self, feedback: dict) -> None:
        pass

    def disconnect(self) -> None:
        # Disable gripper torque before closing the port.
        if self._port_handler is not None and self._packet_handler is not None:
            gripper_mid = self.config.motor_ids[self.config.gripper_index]
            self._packet_handler.write1ByteTxRx(
                self._port_handler, gripper_mid, ADDR_TORQUE_ENABLE, 0
            )
        if self._sync_reader is not None:
            self._sync_reader.clearParam()
            self._sync_reader = None
        if self._port_handler is not None:
            self._port_handler.closePort()
            self._port_handler = None
        self._packet_handler = None
        self._is_connected = False
        logger.info("OmyLeader disconnected.")

    # ---- Internal ----

    def _apply_gripper_spring(self, positions_rad: list[float]) -> None:
        """Apply a spring + damping torque to the gripper motor.

        torque = -stiffness * (pos - neutral) - damping * velocity, converted to
        Goal Current units and clamped for safety.
        """
        grip_idx = self.config.gripper_index
        grip_mid = self.config.motor_ids[grip_idx]
        grip_pos = positions_rad[grip_idx]

        raw_vel, res, err = self._packet_handler.read4ByteTxRx(
            self._port_handler, grip_mid, ADDR_PRESENT_VELOCITY
        )
        if res == 0 and err == 0:
            signed_vel = struct.unpack("i", struct.pack("I", raw_vel))[0]
            # X-series velocity unit: 0.0239691227 rev/min -> rad/s
            grip_vel = signed_vel * 0.0239691227 * 2.0 * math.pi / 60.0
        else:
            grip_vel = 0.0

        torque = (
            -self.config.gripper_spring_stiffness * (grip_pos - self.config.gripper_spring_neutral_rad)
            - self.config.gripper_spring_damping * grip_vel
        )

        current_raw = int(torque / XC330_T181_CURRENT_UNIT)
        max_cur = self.config.gripper_spring_max_current
        current_raw = max(-max_cur, min(current_raw, max_cur))

        value = current_raw & 0xFFFF  # signed 16-bit packed as unsigned for the SDK
        self._packet_handler.write2ByteTxRx(
            self._port_handler, grip_mid, ADDR_GOAL_CURRENT, value
        )

    def _read_positions_rad(self) -> list[float]:
        """Sync read Present_Position from all motors, return as radians.

        Retries up to 3 times on transient communication errors (e.g. USB-serial
        glitch). Falls back to the previous reading if all retries fail.
        """
        max_retries = 3
        for attempt in range(max_retries):
            result = self._sync_reader.txRxPacket()
            if result == 0:
                break
            if attempt < max_retries - 1:
                logger.warning(
                    f"Sync read attempt {attempt + 1}/{max_retries} failed: "
                    f"{self._packet_handler.getTxRxResult(result)}, retrying..."
                )
            else:
                if self._prev_rad is not None:
                    logger.warning(
                        f"Sync read failed after {max_retries} attempts, using previous reading"
                    )
                    return list(self._prev_rad)
                raise RuntimeError(
                    f"Sync read failed after {max_retries} attempts (no previous reading): "
                    f"{self._packet_handler.getTxRxResult(result)}"
                )

        positions = []
        for mid in self.config.motor_ids:
            if not self._sync_reader.isAvailable(mid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                if self._prev_rad is not None:
                    logger.warning(f"Motor {mid} data not available, using previous reading")
                    return list(self._prev_rad)
                raise RuntimeError(f"Motor {mid} data not available (no previous reading)")
            raw = self._sync_reader.getData(mid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            signed = struct.unpack("i", struct.pack("I", raw))[0]
            centered = signed - self.config.position_zero_offset
            rad = centered * 2.0 * math.pi / self.config.units_per_revolution
            positions.append(rad)

        alpha = self.config.smoothing_factor
        if alpha > 0 and self._prev_rad is not None:
            for i in range(len(positions)):
                positions[i] = alpha * self._prev_rad[i] + (1 - alpha) * positions[i]
        self._prev_rad = list(positions)

        return positions
