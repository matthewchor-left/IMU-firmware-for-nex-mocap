"""Binary framing for IMU samples forwarded over TCP."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterable

MAGIC = b"XIMU"
VERSION = 2
DEVICE_SAMPLE_SIZE = 29
TCP_SAMPLE_SIZE = 33
HEADER_SIZE = 7  # magic(4) + version(1) + sample_count(2)
USB_HEADER_SIZE = 8  # magic(4) + version(1) + msg_type(1) + payload_len(2)
USB_MSG_IMU_BATCH = 0x01
USB_MSG_SYNC_REQ = 0x02
USB_MSG_SYNC_RESP = 0x03
USB_BATCH_CAPACITY = 16

# Backward-compatible aliases for device-side parsing.
SAMPLE_SIZE = DEVICE_SAMPLE_SIZE
ALIGNED_SAMPLE_SIZE = TCP_SAMPLE_SIZE

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
class TcpImuSample:
    host_timestamp_us: int
    sequence: int
    gx: float
    gy: float
    gz: float
    ax: float
    ay: float
    az: float


# Backward-compatible alias.
AlignedImuSample = TcpImuSample


def parse_device_sample(data: bytes, offset: int = 0) -> ImuSample:
    if len(data) < offset + DEVICE_SAMPLE_SIZE:
        raise ValueError(
            f"need at least {DEVICE_SAMPLE_SIZE} bytes at offset {offset}, "
            f"got {len(data) - offset}"
        )

    timestamp_us, = struct.unpack_from("<I", data, offset)
    sequence = data[offset + 4]
    gx, gy, gz, ax, ay, az = struct.unpack_from("<6f", data, offset + 5)
    return ImuSample(timestamp_us, sequence, gx, gy, gz, ax, ay, az)


def parse_sample(data: bytes, offset: int = 0) -> ImuSample:
    return parse_device_sample(data, offset)


def parse_samples(payload: bytes) -> list[ImuSample]:
    if len(payload) % DEVICE_SAMPLE_SIZE != 0:
        raise ValueError(
            f"payload length {len(payload)} is not a multiple of {DEVICE_SAMPLE_SIZE}"
        )

    return [
        parse_device_sample(payload, offset)
        for offset in range(0, len(payload), DEVICE_SAMPLE_SIZE)
    ]


def parse_tcp_sample(data: bytes, offset: int = 0) -> TcpImuSample:
    if len(data) < offset + TCP_SAMPLE_SIZE:
        raise ValueError(
            f"need at least {TCP_SAMPLE_SIZE} bytes at offset {offset}, "
            f"got {len(data) - offset}"
        )

    host_timestamp_us, = struct.unpack_from("<Q", data, offset)
    sequence = data[offset + 8]
    gx, gy, gz, ax, ay, az = struct.unpack_from("<6f", data, offset + 9)
    return TcpImuSample(host_timestamp_us, sequence, gx, gy, gz, ax, ay, az)


def parse_aligned_sample(data: bytes, offset: int = 0) -> TcpImuSample:
    return parse_tcp_sample(data, offset)


def parse_tcp_samples(payload: bytes) -> list[TcpImuSample]:
    if len(payload) % TCP_SAMPLE_SIZE != 0:
        raise ValueError(
            f"payload length {len(payload)} is not a multiple of {TCP_SAMPLE_SIZE}"
        )

    return [
        parse_tcp_sample(payload, offset)
        for offset in range(0, len(payload), TCP_SAMPLE_SIZE)
    ]


def parse_aligned_samples(payload: bytes) -> list[TcpImuSample]:
    return parse_tcp_samples(payload)


def encode_tcp_payload(samples: list[TcpImuSample]) -> bytes:
    payload = bytearray(len(samples) * TCP_SAMPLE_SIZE)
    for index, sample in enumerate(samples):
        offset = index * TCP_SAMPLE_SIZE
        struct.pack_into(
            "<QB6f",
            payload,
            offset,
            sample.host_timestamp_us,
            sample.sequence,
            sample.gx,
            sample.gy,
            sample.gz,
            sample.ax,
            sample.ay,
            sample.az,
        )
    return bytes(payload)


def build_aligned_payload(device_payload: bytes, mapper) -> bytes:
    samples = parse_samples(device_payload)
    payload = bytearray(len(samples) * TCP_SAMPLE_SIZE)
    for index, sample in enumerate(samples):
        host_timestamp_us = mapper.map_device_us(sample.timestamp_us)
        if host_timestamp_us is None:
            raise RuntimeError("clock mapper is not calibrated")

        offset = index * TCP_SAMPLE_SIZE
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


def encode_usb_frame(msg_type: int, payload: bytes, version: int = 1) -> bytes:
    if len(payload) > 0xFFFF:
        raise ValueError(f"USB payload too large: {len(payload)}")
    return (
        MAGIC
        + struct.pack("<BBH", version, msg_type, len(payload))
        + payload
    )


def encode_usb_sync_request(request_id: int) -> bytes:
    return encode_usb_frame(USB_MSG_SYNC_REQ, struct.pack("<I", request_id))


def encode_frame(payload: bytes) -> bytes:
    if len(payload) % TCP_SAMPLE_SIZE != 0:
        raise ValueError(
            f"payload length {len(payload)} is not a multiple of {TCP_SAMPLE_SIZE}"
        )

    sample_count = len(payload) // TCP_SAMPLE_SIZE
    if sample_count > 0xFFFF:
        raise ValueError(f"too many samples in one frame: {sample_count}")

    return MAGIC + struct.pack("<BH", VERSION, sample_count) + payload


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
    samples: list[TcpImuSample]


@dataclass(frozen=True, slots=True)
class UsbMessage:
    version: int
    msg_type: int
    payload: bytes


class UsbFrameStream:
    """Incremental parser for framed USB CDC messages from the device."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[UsbMessage]:
        self._buffer.extend(data)
        messages: list[UsbMessage] = []

        while True:
            if len(self._buffer) < USB_HEADER_SIZE:
                break

            if bytes(self._buffer[:4]) != MAGIC:
                magic_at = self._buffer.find(MAGIC, 1)
                if magic_at < 0:
                    if len(self._buffer) > 3:
                        del self._buffer[:-3]
                    break
                del self._buffer[:magic_at]
                if len(self._buffer) < USB_HEADER_SIZE:
                    break

            version = self._buffer[4]
            msg_type = self._buffer[5]
            payload_len = int.from_bytes(self._buffer[6:8], "little")
            frame_len = USB_HEADER_SIZE + payload_len
            if len(self._buffer) < frame_len:
                break

            payload = bytes(self._buffer[USB_HEADER_SIZE:frame_len])
            del self._buffer[:frame_len]
            messages.append(UsbMessage(version=version, msg_type=msg_type, payload=payload))

        return messages


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
            if version != VERSION:
                del self._buffer[:4]
                continue

            sample_count = int.from_bytes(self._buffer[5:7], "little")
            frame_len = HEADER_SIZE + sample_count * TCP_SAMPLE_SIZE
            if len(self._buffer) < frame_len:
                break

            payload = bytes(self._buffer[HEADER_SIZE:frame_len])
            del self._buffer[:frame_len]
            frames.append(Frame(samples=parse_tcp_samples(payload)))

        return frames


def dropped_samples_in_batch(
    previous: int | None, samples: Iterable[ImuSample | TcpImuSample]
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
