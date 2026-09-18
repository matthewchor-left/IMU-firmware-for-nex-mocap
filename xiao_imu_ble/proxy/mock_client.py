"""Synthetic IMU source for debugging the TCP stream without hardware."""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections.abc import Awaitable, Callable

from .protocol import TcpImuSample, encode_tcp_payload

DEFAULT_MOCK_RATE_HZ = 208.0
DEFAULT_MOCK_BATCH_SIZE = 6
GRAVITY_M_S2 = 9.80665


class MockImuClient:
    def __init__(
        self,
        on_batch: Callable[[bytes], Awaitable[None]],
        *,
        sample_rate_hz: float = DEFAULT_MOCK_RATE_HZ,
        batch_size: int = DEFAULT_MOCK_BATCH_SIZE,
    ) -> None:
        if sample_rate_hz <= 0:
            raise ValueError("mock sample rate must be greater than zero")
        if not 1 <= batch_size <= 0xFFFF:
            raise ValueError("mock batch size must be between 1 and 65535")

        self._on_batch = on_batch
        self._sample_rate_hz = sample_rate_hz
        self._batch_size = batch_size
        self._stop_event = asyncio.Event()

    async def run(self) -> None:
        period_ns = round(1_000_000_000 / self._sample_rate_hz)
        next_sample_ns = time.monotonic_ns()
        sequence = 0
        sample_index = 0

        logging.info(
            "mock IMU started: rate=%.1f Hz batch_size=%d",
            self._sample_rate_hz,
            self._batch_size,
        )

        while not self._stop_event.is_set():
            samples: list[TcpImuSample] = []
            for _ in range(self._batch_size):
                elapsed_s = sample_index / self._sample_rate_hz
                samples.append(
                    TcpImuSample(
                        host_timestamp_us=next_sample_ns // 1_000,
                        sequence=sequence,
                        gx=30.0 * math.sin(2.0 * math.pi * 0.5 * elapsed_s),
                        gy=20.0 * math.sin(2.0 * math.pi * 0.7 * elapsed_s + 0.8),
                        gz=10.0 * math.sin(2.0 * math.pi * 0.3 * elapsed_s + 1.6),
                        ax=1.5 * math.sin(2.0 * math.pi * 0.8 * elapsed_s),
                        ay=1.0 * math.sin(2.0 * math.pi * 0.6 * elapsed_s + 0.4),
                        az=GRAVITY_M_S2
                        + 0.5 * math.sin(2.0 * math.pi * 0.4 * elapsed_s),
                    )
                )
                sequence = (sequence + 1) & 0xFF
                sample_index += 1
                next_sample_ns += period_ns

            await self._on_batch(encode_tcp_payload(samples))

            delay_s = (next_sample_ns - time.monotonic_ns()) / 1_000_000_000
            if delay_s <= 0:
                await asyncio.sleep(0)
                continue

            try:
                await asyncio.wait_for(self._stop_event.wait(), timeout=delay_s)
            except asyncio.TimeoutError:
                pass

    async def stop(self) -> None:
        self._stop_event.set()
