"""MSP encode/decode must match the official clients byte-for-byte."""

import struct

import pytest

from pluto_swarm.hal import msp_protocol as msp

# Reference packets produced by plutocontrol 2.0.0 Pluto.create_packet_msp()
# (captured once; the live comparison below runs too if plutocontrol is installed).
REF_RC_DEFAULT = "244d3c10c8dc05dc05dc05dc05dc05e803dc05e803d8"
REF_TAKEOFF = "244d3c01d901d9"    # MSP_SET_COMMAND, 1-byte payload = 1
REF_ATTITUDE_REQ = "244d3c006c6c"


def test_rc_matches_reference():
    assert msp.encode_set_raw_rc([1500, 1500, 1500, 1500, 1500, 1000, 1500, 1000]).hex() == REF_RC_DEFAULT


def test_set_command_matches_reference():
    assert msp.encode_set_command(1).hex() == REF_TAKEOFF


def test_request_matches_reference():
    assert msp.encode_request(msp.MSP_ATTITUDE).hex() == REF_ATTITUDE_REQ


def test_live_plutocontrol_byte_for_byte():
    pc = pytest.importorskip("plutocontrol.Pluto")
    p = pc.Pluto.__new__(pc.Pluto)          # no socket needed for packet building
    cases = [
        (msp.MSP_SET_RAW_RC, [1500, 1600, 1800, 1400, 1500, 1000, 1500, 1500],
         lambda v: msp.encode_set_raw_rc(v)),
        (msp.MSP_SET_COMMAND, [1], lambda v: msp.encode_set_command(v[0])),
        (msp.MSP_SET_COMMAND, [2], lambda v: msp.encode_set_command(v[0])),
        (msp.MSP_ALTITUDE, [], lambda v: msp.encode_request(msp.MSP_ALTITUDE)),
        (msp.MSP_ANALOG, [], lambda v: msp.encode_request(msp.MSP_ANALOG)),
    ]
    for cmd, payload, ours in cases:
        assert ours(payload).hex() == p.create_packet_msp(cmd, payload), (cmd, payload)


def test_negative_angles_decode_signed():
    # The ROS package bug: -1.0 deg came back as ~6553. We must get -1.0.
    pkt = msp.encode_attitude_reply(-0.0174533, -0.5, 0.0)
    (p,) = list(msp.MspParser().feed(pkt))
    a = msp.decode_attitude(p.payload)
    assert a.roll_rad == pytest.approx(-0.0174533, abs=2e-3)
    assert a.pitch_rad == pytest.approx(-0.5, abs=2e-3)


def test_altitude_roundtrip_negative():
    (p,) = list(msp.MspParser().feed(msp.encode_altitude_reply(-0.25, -0.4)))
    a = msp.decode_altitude(p.payload)
    assert a.alt_m == pytest.approx(-0.25) and a.vario_m_s == pytest.approx(-0.4)


def test_analog_v1_and_legacy():
    (p,) = list(msp.MspParser().feed(msp.encode_analog_reply(3.87)))
    assert msp.decode_analog(p.payload).battery_v == pytest.approx(3.87, abs=1e-3)
    assert msp.decode_analog(bytes([39, 0, 0, 0, 0, 0, 0])).battery_v == pytest.approx(3.9)


def test_parser_handles_split_garbage_and_bad_checksum():
    good = msp.encode_altitude_reply(1.0, 0.0)
    bad = bytearray(msp.encode_attitude_reply(0, 0, 0))
    bad[-1] ^= 0xFF
    stream = b"junk$MD debug text" + bytes(bad) + good + good
    parser = msp.MspParser()
    out = []
    for i in range(0, len(stream), 3):          # feed in tiny chunks
        out += list(parser.feed(stream[i:i + 3]))
    assert [p.cmd for p in out] == [msp.MSP_ALTITUDE, msp.MSP_ALTITUDE]
    assert parser.bad_checksums == 1


def test_rc_payload_layout():
    pkt = msp.encode_set_raw_rc([1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700])
    assert pkt[3] == 16 and pkt[4] == msp.MSP_SET_RAW_RC
    assert list(struct.unpack("<8H", pkt[5:21])) == [1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700]
