"""MSP v1 protocol for Pluto (MagisV2 firmware) - pure functions, no sockets.

Packet on the wire (all little-endian):

    '$' 'M' <dir> | size (1 B) | cmd (1 B) | payload (size B) | checksum (1 B)

    dir  = '<' ground -> drone (request / set)
           '>' drone -> ground (reply)
           '!' drone -> ground (error reply)
    checksum = XOR of size, cmd and every payload byte

Verified against: plutocontrol 2.0.0 (Pluto.create_packet_msp), DronaAviation
pluto_ros2_package (Protocol.createPacketMSP) and MagisV2 serial_msp.cpp.

Fixes vs the provided libraries:
  * replies are decoded with struct and proper signed types (the ROS package's
    read16 never sign-extends, so -1.0 deg arrived as ~6553);
  * replies are found by a real stream parser with checksum checking, not by
    searching for the first byte equal to the command number.

This module is shared by the real-drone link AND the simulator, so both speak
exactly the same bytes.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Iterator, List, Optional

# ---- command numbers (MagisV2 serial_msp.cpp) --------------------------------
MSP_FC_VERSION = 3
MSP_RAW_IMU = 102
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_SET_RAW_RC = 200
MSP_ACC_CALIBRATION = 205
MSP_SET_MOTOR = 214
MSP_SET_POS = 216
MSP_SET_COMMAND = 217
MSP_SET_ACC_TRIM = 239
MSP_ACC_TRIM = 240
MSP_EEPROM_WRITE = 250

DIR_TO_DRONE = ord("<")
DIR_FROM_DRONE = ord(">")
DIR_ERROR = ord("!")


def checksum(size: int, cmd: int, payload: bytes) -> int:
    c = (size ^ cmd) & 0xFF
    for b in payload:
        c ^= b
    return c


def encode(cmd: int, payload: bytes = b"", direction: int = DIR_TO_DRONE) -> bytes:
    """Build one MSP packet."""
    if len(payload) > 255:
        raise ValueError("MSP v1 payload max 255 bytes")
    size = len(payload)
    return (bytes([ord("$"), ord("M"), direction, size, cmd & 0xFF])
            + payload + bytes([checksum(size, cmd & 0xFF, payload)]))


# ---- ground -> drone ---------------------------------------------------------

def encode_set_raw_rc(channels: List[int]) -> bytes:
    """8 RC channels, uint16 each: roll, pitch, throttle, yaw, aux1..aux4."""
    if len(channels) != 8:
        raise ValueError("expected 8 RC channels")
    return encode(MSP_SET_RAW_RC, struct.pack("<8H", *[int(c) for c in channels]))


def encode_set_command(command: int) -> bytes:
    """Take-off (1) / land (2). ONE payload byte, exactly like both official
    clients; the firmware stores it in a uint8_t, so this is what it expects."""
    return encode(MSP_SET_COMMAND, bytes([command & 0xFF]))


def encode_request(cmd: int) -> bytes:
    """Ask the drone for telemetry (empty payload)."""
    return encode(cmd)


def encode_set_acc_trim(pitch: int, roll: int) -> bytes:
    """Firmware reads pitch first, then roll (serial_msp.cpp MSP_SET_ACC_TRIM)."""
    return encode(MSP_SET_ACC_TRIM, struct.pack("<hh", pitch, roll))


# ---- drone -> ground replies (and their encoders, used by the simulator) -----

@dataclass
class Attitude:
    roll_rad: float
    pitch_rad: float
    yaw_rad: float


@dataclass
class Altitude:
    alt_m: float
    vario_m_s: float


@dataclass
class Analog:
    battery_v: float
    current_ma: Optional[int] = None
    soc_percent: Optional[int] = None


def decode_attitude(p: bytes) -> Attitude:
    roll_dd, pitch_dd, heading_deg = struct.unpack_from("<hhh", p)
    return Attitude(math.radians(roll_dd / 10.0), math.radians(pitch_dd / 10.0),
                    math.radians(heading_deg))


def encode_attitude_reply(roll_rad: float, pitch_rad: float, yaw_rad: float) -> bytes:
    heading = int(round(math.degrees(yaw_rad))) % 360
    payload = struct.pack("<hhh", int(round(math.degrees(roll_rad) * 10)),
                          int(round(math.degrees(pitch_rad) * 10)), heading)
    return encode(MSP_ATTITUDE, payload, DIR_FROM_DRONE)


def decode_altitude(p: bytes) -> Altitude:
    alt_cm, vario_cm_s = struct.unpack_from("<ih", p)
    return Altitude(alt_cm / 100.0, vario_cm_s / 100.0)


def encode_altitude_reply(alt_m: float, vario_m_s: float) -> bytes:
    payload = struct.pack("<ih", int(round(alt_m * 100)), int(round(vario_m_s * 100)))
    return encode(MSP_ALTITUDE, payload, DIR_FROM_DRONE)


def decode_analog(p: bytes) -> Analog:
    """MagisV2 (protocol v1) sends 10 bytes: vBatComp mV, mAmpRaw, mAhDrawn,
    mAhRemain (uint16 each), soc %, warning mode (uint8 each).
    Older firmware sends the classic 7-byte layout with vbat in 0.1 V."""
    if len(p) >= 10:
        vbat_mv, ma, _drawn, _remain, soc, _warn = struct.unpack_from("<HHHHBB", p)
        return Analog(vbat_mv / 1000.0, ma, soc)
    if len(p) >= 1:
        return Analog(p[0] / 10.0)
    raise ValueError("empty MSP_ANALOG payload")


def encode_analog_reply(battery_v: float, current_ma: int = 0, soc: int = 100) -> bytes:
    payload = struct.pack("<HHHHBB", int(battery_v * 1000), current_ma, 0, 0,
                          max(0, min(100, soc)), 0)
    return encode(MSP_ANALOG, payload, DIR_FROM_DRONE)


# ---- stream parser ------------------------------------------------------------

@dataclass
class Packet:
    direction: int
    cmd: int
    payload: bytes


class MspParser:
    """Feed it raw bytes from a socket (any chunk size); it yields complete,
    checksum-valid packets. Garbage and debug text between packets is skipped."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.bad_checksums = 0

    def feed(self, data: bytes) -> Iterator[Packet]:
        self._buf += data
        while True:
            start = self._buf.find(b"$M")
            if start < 0:
                # keep a trailing '$' in case the 'M' arrives next chunk
                self._buf = self._buf[-1:] if self._buf.endswith(b"$") else bytearray()
                return
            if start:
                del self._buf[:start]
            if len(self._buf) < 5:
                return
            direction, size, cmd = self._buf[2], self._buf[3], self._buf[4]
            if direction not in (DIR_TO_DRONE, DIR_FROM_DRONE, DIR_ERROR):
                del self._buf[:2]           # e.g. "$MD" debug text - skip it
                continue
            total = 5 + size + 1
            if len(self._buf) < total:
                return
            payload = bytes(self._buf[5:5 + size])
            if self._buf[5 + size] != checksum(size, cmd, payload):
                self.bad_checksums += 1
                del self._buf[:2]
                continue
            del self._buf[:total]
            yield Packet(direction, cmd, payload)
