import io
import json
import struct

import pytest

from tools import flight_log_receive as flog


class FakePort:
    def __init__(self, data: bytes) -> None:
        self._rx = io.BytesIO(data)
        self.writes: list[bytes] = []

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def read(self, length: int) -> bytes:
        return self._rx.read(length)

    def readline(self) -> bytes:
        return self._rx.readline()


def test_export_block_parser_validates_crc() -> None:
    payload = b"abc123"
    header = flog.EXPORT_HEADER.pack(
        flog.EXPORT_BLOCK_MAGIC,
        1,
        flog.EXPORT_HEADER.size,
        7,
        42,
        len(payload),
        flog.EXPORT_BLOCK_LAST,
        flog.crc32(payload),
    )
    block = flog.read_export_block(io.BytesIO(header + payload))

    assert block.seq == 7
    assert block.offset == 42
    assert block.payload == payload
    assert block.flags == flog.EXPORT_BLOCK_LAST


def test_export_block_parser_rejects_crc_mismatch() -> None:
    payload = b"abc123"
    header = flog.EXPORT_HEADER.pack(
        flog.EXPORT_BLOCK_MAGIC,
        1,
        flog.EXPORT_HEADER.size,
        0,
        0,
        len(payload),
        0,
        0x12345678,
    )

    with pytest.raises(flog.FlightLogError, match="crc mismatch"):
        flog.read_export_block(io.BytesIO(header + payload))


def test_begin_line_requires_flight_log_magic() -> None:
    begin = flog.parse_begin_line(
        "FLOG BEGIN version=1 block_magic=0x31424C46 total=4096 sectors=1"
    )

    assert begin.total == 4096


def test_end_line_requires_done_and_matching_counts() -> None:
    end = flog.read_until_end(
        FakePort(b"FLOG END reason=done sent=4096 total=4096\r\n"),
        received=4096,
        expected_total=4096,
    )

    assert end.reason == "done"
    assert end.sent == 4096
    assert end.total == 4096

    with pytest.raises(flog.FlightLogError, match="reason=error"):
        flog.read_until_end(
            FakePort(b"FLOG END reason=error sent=128 total=4096\r\n"),
            received=128,
            expected_total=4096,
        )

    with pytest.raises(flog.FlightLogError, match="export end mismatch"):
        flog.read_until_end(
            FakePort(b"FLOG END reason=done sent=128 total=4096\r\n"),
            received=4096,
            expected_total=4096,
        )


def test_parse_record_and_csv_fields(tmp_path) -> None:
    packed = make_record()

    row = flog.parse_record(packed)
    assert row is not None
    assert row["sequence"] == 3
    assert row["raw_gyro_z"] == 7
    assert row["rc_ch8_us"] == 1007
    assert row["vel_pid_d_m_s2_1"] == 17.0
    assert row["ctrl_pos_p_m_s2_0"] == 19.0
    assert row["ctrl_omega_cmd_rad_s_1"] == 39.0

    csv_path = tmp_path / "out.csv"
    flog.write_csv(csv_path, [row])
    text = csv_path.read_text(encoding="utf-8")
    assert "timestamp_us" in text
    assert "vel_pid_out_m_s2_0" in text


def make_record() -> bytes:
    values = []
    values.extend([flog.RECORD_MAGIC, 1, flog.RECORD_SIZE, 3, 0, 1000, 12, 99])
    values.extend([1, 2, 3, 4, 5, 6, 7])
    values.extend([25.0, 0.1, 0.2, 0.3, 10.0, 11.0, 12.0])
    values.extend([1.0, 2.0, 3.0])
    values.extend([1000 + i for i in range(8)])
    values.extend([1200, 1441, 1877, 1300, 1310])
    values.extend([1, 1, 1, 1])
    values.extend([float(i) for i in range(41)])
    values.append(0)
    packed_without_crc = flog.RECORD_STRUCT.pack(*values)
    crc = flog.crc32(packed_without_crc[:-4] + b"\x00\x00\x00\x00")
    values[-1] = crc
    return flog.RECORD_STRUCT.pack(*values)


def make_sector_header() -> bytes:
    params = [float(i) for i in range(len(flog.PARAM_NAMES))]
    reserved = b"\x00" * 60
    prefix = flog.SECTOR_HEADER_PREFIX.pack(
        flog.SECTOR_MAGIC,
        1,
        flog.SECTOR_HEADER_SIZE,
        flog.SECTOR_SIZE,
        flog.RECORD_SIZE,
        123,
        5,
        0,
        250,
        0x2000,
        0x3FC000,
        100,
        flog.PARAMS_STRUCT.size,
        0,
    )
    header = bytearray(prefix + flog.PARAMS_STRUCT.pack(*params) + reserved)
    assert len(header) == flog.SECTOR_HEADER_SIZE
    crc = flog.crc32(bytes(header))
    struct.pack_into("<I", header, 52, crc)
    return bytes(header)


def make_export_block(seq: int, offset: int, payload: bytes, flags: int = 0) -> bytes:
    header = flog.EXPORT_HEADER.pack(
        flog.EXPORT_BLOCK_MAGIC,
        1,
        flog.EXPORT_HEADER.size,
        seq,
        offset,
        len(payload),
        flags,
        flog.crc32(payload),
    )
    return header + payload


def test_sector_header_and_flash_image_parse() -> None:
    image = make_sector_header() + (b"\xFF" * (flog.SECTOR_SIZE - flog.SECTOR_HEADER_SIZE))

    sectors, records, errors = flog.parse_flash_image(image)

    assert errors == []
    assert records == []
    assert sectors[0]["session_id"] == 123
    assert sectors[0]["params"]["vel_loop_x_kp"] == 10.0


def test_receive_dump_writes_bin_csv_and_meta(tmp_path) -> None:
    image = bytearray(b"\xFF" * flog.SECTOR_SIZE)
    image[:flog.SECTOR_HEADER_SIZE] = make_sector_header()
    record = make_record()
    image[flog.SECTOR_HEADER_SIZE : flog.SECTOR_HEADER_SIZE + len(record)] = record
    begin = (
        "FLOG BEGIN version=1 block_magic=0x31424C46 total=4096 sectors=1 "
        "sector_size=4096 header_size=256 record_size=284 log_rate=250 baud=57600 "
        "session=123\r\n"
    ).encode("ascii")
    payload = bytes(image)
    end = b"FLOG END reason=done sent=4096 total=4096\r\n"
    stream = begin + make_export_block(0, 0, payload, flog.EXPORT_BLOCK_LAST) + end
    port = FakePort(stream)

    result = flog.receive_dump(port, tmp_path)

    assert port.writes == [b"Sensor_Data:0\r\n", b"FLOG DUMP\r\n"]
    assert result.total_bytes == flog.SECTOR_SIZE
    assert result.blocks == 1
    assert result.records == 1
    assert result.sectors == 1
    assert result.bin_path.read_bytes() == payload
    assert "ctrl_total_force_n" in result.csv_path.read_text(encoding="utf-8")
    meta = json.loads(result.meta_path.read_text(encoding="utf-8"))
    assert meta["record_size"] == flog.RECORD_SIZE
    assert meta["baud"] == 57600
    assert meta["record_count"] == 1
    assert meta["end"]["reason"] == "done"


def test_receive_dump_rejects_missing_end_line(tmp_path) -> None:
    payload = b"\xFF" * flog.SECTOR_SIZE
    begin = (
        "FLOG BEGIN version=1 block_magic=0x31424C46 total=4096 sectors=1\r\n"
    ).encode("ascii")
    stream = begin + make_export_block(0, 0, payload, flog.EXPORT_BLOCK_LAST)
    port = FakePort(stream)

    with pytest.raises(flog.FlightLogError, match="timeout waiting for FLOG END"):
        flog.receive_dump(port, tmp_path)


def test_receive_dump_cancel_sends_flog_cancel(tmp_path) -> None:
    begin = (
        "FLOG BEGIN version=1 block_magic=0x31424C46 total=4096 sectors=1\r\n"
    ).encode("ascii")
    port = FakePort(begin)

    with pytest.raises(flog.FlightLogError, match="cancelled"):
        flog.receive_dump(port, tmp_path, should_cancel=lambda: True)

    assert port.writes == [b"Sensor_Data:0\r\n", b"FLOG DUMP\r\n", b"FLOG CANCEL\r\n"]
