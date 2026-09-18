"""Accumulate device batches into larger TCP frames."""

from __future__ import annotations

import time

from .protocol import TcpImuSample, encode_frame, encode_tcp_payload, parse_aligned_samples


class TcpRebatcher:
    """Merge incoming aligned batches into larger TCP frames."""

    def __init__(self, sample_target: int, max_wait_s: float = 0.0) -> None:
        if sample_target < 0:
            raise ValueError("sample_target must be >= 0")
        if max_wait_s < 0:
            raise ValueError("max_wait_s must be >= 0")

        self.sample_target = sample_target
        self.max_wait_s = max_wait_s
        self._samples: list[TcpImuSample] = []
        self._buffer_started_at: float | None = None

    @property
    def enabled(self) -> bool:
        return self.sample_target > 0

    def ingest(self, payload: bytes) -> list[bytes]:
        if not self.enabled:
            return [encode_frame(payload)]

        self._samples.extend(parse_aligned_samples(payload))
        return self._emit_full_batches()

    def flush_if_stale(self) -> list[bytes]:
        if not self.enabled or not self._samples or self.max_wait_s <= 0:
            return []
        if self._buffer_started_at is None:
            return []
        if time.monotonic() - self._buffer_started_at < self.max_wait_s:
            return []
        return self._emit_partial()

    def flush(self) -> list[bytes]:
        if not self.enabled or not self._samples:
            return []
        return self._emit_partial()

    def _emit_full_batches(self) -> list[bytes]:
        frames: list[bytes] = []
        while len(self._samples) >= self.sample_target:
            chunk = self._samples[: self.sample_target]
            del self._samples[: self.sample_target]
            frames.append(encode_frame(encode_tcp_payload(chunk)))
            self._reset_buffer_timer()

        if self._samples and self._buffer_started_at is None:
            self._buffer_started_at = time.monotonic()
        if not self._samples:
            self._buffer_started_at = None
        return frames

    def _emit_partial(self) -> list[bytes]:
        chunk = self._samples
        self._samples = []
        self._buffer_started_at = None
        if not chunk:
            return []
        return [encode_frame(encode_tcp_payload(chunk))]

    def _reset_buffer_timer(self) -> None:
        if self._samples:
            self._buffer_started_at = time.monotonic()
        else:
            self._buffer_started_at = None
