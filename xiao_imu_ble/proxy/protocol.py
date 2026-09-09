"""Binary framing for IMU samples forwarded over TCP."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterable

MAGIC = b"XIMU"
VERSION = 1
VERSION_ALIGNED = 2
SAMPLE_SIZE = 29
ALIGNED_SAMPLE_SIZE = 33
HEADER_SIZE = 7  # magic(4) + version(1) + sample_count(2)

SERVICE_UUID = "bfe2b6e1-0003-4583-926c-c39f476f7a34"
IMU_CHAR_UUID = "bfe2b6e1-0004-4583-926c-c39f476f7a34"
SYNC_CHAR_UUID = "bfe2b6e1-0005-4583-926c-c39f476f7a34"
DEFAULT_DEVICE_NAME = "XiaoIMU"


@dataclass(frozen=True, slots=True)
class ImuSample:
    timestamp_us: int
    sequence: int
    gx: float
    gy: float
    gz: float
    ax: float
    ay: float
    az: float


@dataclass(frozen=True, slots=True)
class AlignedImuSample:
    host_timestamp_us: int
    sequence: int
    gx: float
    gy: float
    gz: float
    ax: float
    ay: float
    az: float


def sample_size_for_version(version: int) -> int:
    if version == VERSION:
        return SAMPLE_SIZE
    if version == VERSION_ALIGNED:
        return ALIGNED_SAMPLE_SIZE
    raise ValueError(f"unsupported protocol version: {version}")


def parse_sample(data: bytes, offset: int = 0) -> ImuSample:
    if len(data) < offset + SAMPLE_SIZE:
        raise ValueError(
            f"need at least {SAMPLE_SIZE} bytes at offset {offset}, got {len(data) - offset}"
        )

    timestamp_us, = struct.unpack_from("<I", data, offset)
    sequence = data[offset + 4]
    gx, gy, gz, ax, ay, az = struct.unpack_from("<6f", data, offset + 5)
    return ImuSample(timestamp_us, sequence, gx, gy, gz, ax, ay, az)


def parse_samples(payload: bytes) -> list[ImuSample]:
    if len(payload) % SAMPLE_SIZE != 0:
        raise ValueError(f"payload length {len(payload)} is not a multiple of {SAMPLE_SIZE}")

    return [parse_sample(payload, offset) for offset in range(0, len(payload), SAMPLE_SIZE)]


def parse_aligned_sample(data: bytes, offset: int = 0) -> AlignedImuSample:
    if len(data) < offset + ALIGNED_SAMPLE_SIZE:
        raise ValueError(
            f"need at least {ALIGNED_SAMPLE_SIZE} bytes at offset {offset}, got {len(data) - offset}"
        )

    host_timestamp_us, = struct.unpack_from("<Q", data, offset)
    sequence = data[offset + 8]
    gx, gy, gz, ax, ay, az = struct.unpack_from("<6f", data, offset + 9)
    return AlignedImuSample(host_timestamp_us, sequence, gx, gy, gz, ax, ay, az)


def parse_aligned_samples(payload: bytes) -> list[AlignedImuSample]:
    if len(payload) % ALIGNED_SAMPLE_SIZE != 0:
        raise ValueError(
            f"payload length {len(payload)} is not a multiple of {ALIGNED_SAMPLE_SIZE}"
        )

    return [
        parse_aligned_sample(payload, offset)
        for offset in range(0, len(payload), ALIGNED_SAMPLE_SIZE)
    ]


def build_aligned_payload(device_payload: bytes, mapper) -> bytes:
    samples = parse_samples(device_payload)
    payload = bytearray(len(samples) * ALIGNED_SAMPLE_SIZE)
    for index, sample in enumerate(samples):
        host_timestamp_us = mapper.map_device_us(sample.timestamp_us)
        if host_timestamp_us is None:
            raise RuntimeError("clock mapper is not calibrated")

        offset = index * ALIGNED_SAMPLE_SIZE
        struct.pack_into(
            "<QB6f",
            payload,
            offset,
            host_timestamp_us,
            sample.sequence,
            sample.gx,
            sample.gy,
            sample.gz,
            sample.ax,
            sample.ay,
            sample.az,
        )
    return bytes(payload)


def encode_frame(payload: bytes, version: int = VERSION) -> bytes:
    sample_size = sample_size_for_version(version)
    if len(payload) % sample_size != 0:
        raise ValueError(f"payload length {len(payload)} is not a multiple of {sample_size}")

    sample_count = len(payload) // sample_size
    if sample_count > 0xFFFF:
        raise ValueError(f"too many samples in one frame: {sample_count}")

    return MAGIC + struct.pack("<BH", version, sample_count) + payload


def sequence_gap(previous: int | None, current: int) -> int | None:
    """Return number of dropped samples between two sequence values, if any."""
    if previous is None:
        return None

    diff = (current - previous) & 0xFF
    if diff <= 1:
        return None
    if diff > 128:
        # Likely a source reset rather than a gap.
        return None

    return diff - 1


@dataclass(slots=True)
class Frame:
    version: int
    samples: list[ImuSample] | list[AlignedImuSample]


class FrameStream:
    """Incremental parser for length-prefixed IMU frames from a TCP byte stream."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[Frame]:
        self._buffer.extend(data)
        frames: list[Frame] = []

        while True:
            if len(self._buffer) < HEADER_SIZE:
                break

            if bytes(self._buffer[:4]) != MAGIC:
                magic_at = self._buffer.find(MAGIC, 1)
                if magic_at < 0:
                    if len(self._buffer) > 3:
                        del self._buffer[:-3]
                    break
                del self._buffer[:magic_at]
                if len(self._buffer) < HEADER_SIZE:
                    break

            version = self._buffer[4]
            sample_count = int.from_bytes(self._buffer[5:7], "little")
            try:
                sample_size = sample_size_for_version(version)
            except ValueError:
                del self._buffer[:4]
                continue

            frame_len = HEADER_SIZE + sample_count * sample_size
            if len(self._buffer) < frame_len:
                break

            payload = bytes(self._buffer[HEADER_SIZE:frame_len])
            del self._buffer[:frame_len]
            if version == VERSION_ALIGNED:
                frames.append(Frame(version=version, samples=parse_aligned_samples(payload)))
            else:
                frames.append(Frame(version=version, samples=parse_samples(payload)))

        return frames


def dropped_samples_in_batch(
    previous: int | None, samples: Iterable[ImuSample]
) -> tuple[int | None, int]:
    """Update sequence tracker and return (new_previous, total_dropped)."""
    dropped = 0
    last = previous

    for sample in samples:
        gap = sequence_gap(last, sample.sequence)
        if gap is not None:
            dropped += gap
        last = sample.sequence

    return last, dropped
