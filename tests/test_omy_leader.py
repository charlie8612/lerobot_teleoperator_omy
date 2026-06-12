import math

import pytest

from lerobot_teleoperator_omy import OmyLeader, OmyLeaderConfig
from lerobot_teleoperator_omy.omy_leader import (
    ADDR_OPERATING_MODE,
    ADDR_TORQUE_ENABLE,
    OP_MODE_CURRENT,
    OP_MODE_EXTENDED_POSITION,
    raw_to_rad,
)


# --- pure conversion -------------------------------------------------------


def test_raw_to_rad_at_zero_offset_is_zero():
    assert raw_to_rad(2048, zero_offset=2048, units_per_revolution=4096) == 0.0


def test_raw_to_rad_quarter_turn():
    # 1024 units above the 2048 center = quarter revolution = pi/2 rad
    assert raw_to_rad(3072, 2048, 4096) == pytest.approx(math.pi / 2)


def test_raw_to_rad_negative():
    assert raw_to_rad(1024, 2048, 4096) == pytest.approx(-math.pi / 2)


def test_raw_to_rad_multiturn_beyond_one_revolution():
    # Extended Position mode reports values past 4095; 2048 + 4096 = +1 full extra turn
    assert raw_to_rad(2048 + 4096, 2048, 4096) == pytest.approx(2 * math.pi)


def test_raw_to_rad_handles_signed_negative_raw():
    # raw stored as unsigned 32-bit; a "negative" multi-turn position must decode signed
    raw = (-4096) & 0xFFFFFFFF
    assert raw_to_rad(raw, 0, 4096) == pytest.approx(-2 * math.pi)


# --- config / schema -------------------------------------------------------


def test_config_defaults():
    cfg = OmyLeaderConfig(port="/dev/ttyUSB0")
    assert cfg.baudrate == 4_000_000
    assert cfg.motor_ids == [1, 2, 3, 4, 5, 6, 7]
    assert cfg.gripper_index == 6
    assert cfg.arm_operating_mode == OP_MODE_EXTENDED_POSITION
    assert cfg.position_zero_offset == 2048
    assert cfg.units_per_revolution == 4096


def test_action_features_keys():
    leader = OmyLeader(OmyLeaderConfig(port="/dev/ttyUSB0"))
    assert list(leader.action_features) == [
        "joint_1.pos",
        "joint_2.pos",
        "joint_3.pos",
        "joint_4.pos",
        "joint_5.pos",
        "joint_6.pos",
        "gripper.pos",
    ]


# --- mocked connect: verifies Extended/Current operating modes are written -


class _FakePort:
    def __init__(self, *a, **k):
        pass

    def openPort(self):
        return True

    def setBaudRate(self, baud):
        return True

    def closePort(self):
        return True


class _FakePacket:
    def __init__(self, *a, **k):
        self.writes = []  # (motor_id, address, value)

    def write1ByteTxRx(self, port, mid, addr, value):
        self.writes.append((mid, addr, value))
        return 0, 0

    def write2ByteTxRx(self, port, mid, addr, value):
        return 0, 0

    def read4ByteTxRx(self, port, mid, addr):
        return 0, 0, 0

    def getTxRxResult(self, result):
        return "ok"


class _FakeSyncReader:
    def __init__(self, *a, **k):
        pass

    def addParam(self, mid):
        return True

    def txRxPacket(self):
        return 0

    def isAvailable(self, mid, addr, length):
        return True

    def getData(self, mid, addr, length):
        return 2048

    def clearParam(self):
        pass


def test_connect_sets_extended_mode_on_arm_and_current_on_gripper(monkeypatch):
    import lerobot_teleoperator_omy.omy_leader as mod

    monkeypatch.setattr(mod, "PortHandler", _FakePort)
    fake_packet = _FakePacket()
    monkeypatch.setattr(mod, "PacketHandler", lambda *a, **k: fake_packet)
    monkeypatch.setattr(mod, "GroupSyncRead", _FakeSyncReader)

    leader = OmyLeader(OmyLeaderConfig(port="/dev/ttyUSB0"))
    leader.connect()

    mode_writes = {
        mid: val
        for (mid, addr, val) in fake_packet.writes
        if addr == ADDR_OPERATING_MODE
    }
    # arm motors 1-6 -> Extended, gripper motor 7 -> Current
    for arm_id in range(1, 7):
        assert mode_writes[arm_id] == OP_MODE_EXTENDED_POSITION
    assert mode_writes[7] == OP_MODE_CURRENT

    # all motors had torque disabled before mode change (EEPROM write requires it)
    torque_writes = [
        (mid, val)
        for (mid, addr, val) in fake_packet.writes
        if addr == ADDR_TORQUE_ENABLE
    ]
    assert (1, 0) in torque_writes  # arm motor torque disabled
    assert leader.is_connected
