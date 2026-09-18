"""Runtime statistics for the IMU proxy."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from .protocol import ImuSample, SAMPLE_SIZE, dropped_samples_in_batch


def batch_capacity_for_mtu(mtu: int) -> int:
    if mtu <= 3:
        return 0
    payload_bytes = mtu - 3
    if payload_bytes < SAMPLE_SIZE:
        return 0
    return min(6, payload_bytes // SAMPLE_SIZE)


@dataclass
class ProxyStats:
    mtu: int = 0
    ble_batches: int = 0
    ble_samples: int = 0
    tcp_frames_sent: int = 0
    tcp_bytes_sent: int = 0
    dropped_samples: int = 0
    dropped_tcp_frames: int = 0
    last_sequence: int | None = None
    last_batch_size: int = 0
    batch_sizes: dict[int, int] = field(default_factory=dict)
    session_started_at: float = field(default_factory=time.monotonic)
    last_batch_at: float | None = None

    def reset_session(self, mtu: int) -> None:
        self.mtu = mtu
        self.ble_batches = 0
        self.ble_samples = 0
        self.tcp_frames_sent = 0
        self.tcp_bytes_sent = 0
        self.dropped_samples = 0
        self.dropped_tcp_frames = 0
        self.last_sequence = None
        self.last_batch_size = 0
        self.batch_sizes.clear()
        self.session_started_at = time.monotonic()
        self.last_batch_at = None

    def record_ble_batch(self, samples: list[ImuSample]) -> None:
        count = len(samples)
        self.ble_batches += 1
        self.ble_samples += count
        self.last_batch_size = count
        self.batch_sizes[count] = self.batch_sizes.get(count, 0) + 1
        self.last_batch_at = time.monotonic()

        self.last_sequence, dropped = dropped_samples_in_batch(
            self.last_sequence,
            samples,
        )
        self.dropped_samples += dropped

    def record_tcp_send(self, frame_len: int) -> None:
        self.tcp_frames_sent += 1
        self.tcp_bytes_sent += frame_len

    def record_tcp_drop(self) -> None:
        self.dropped_tcp_frames += 1

    def seconds_since_last_batch(self) -> float | None:
        if self.last_batch_at is None:
            return None
        return time.monotonic() - self.last_batch_at

    def format_summary(self, transport: str = "ble") -> str:
        elapsed = max(time.monotonic() - self.session_started_at, 1e-6)
        batch_summary = ", ".join(
            f"{size}x{count}" for size, count in sorted(self.batch_sizes.items())
        )
        if transport == "usb":
            transport_info = "transport=usb expected_batch=16"
        else:
            expected_batch = batch_capacity_for_mtu(self.mtu)
            transport_info = f"transport=ble mtu={self.mtu} expected_batch={expected_batch}"
        return (
            f"{transport_info} "
            f"input={self.ble_batches / elapsed:.1f} batches/s "
            f"{self.ble_samples / elapsed:.1f} samples/s "
            f"tcp_sent={self.tcp_frames_sent} "
            f"dropped_samples={self.dropped_samples} "
            f"dropped_tcp_frames={self.dropped_tcp_frames} "
            f"batches={{{batch_summary}}}"
        )
