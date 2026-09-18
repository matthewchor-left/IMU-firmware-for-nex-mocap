"""Device clock synchronization and host timestamp mapping."""

from __future__ import annotations

import asyncio
import contextlib
import logging
import struct
import time
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import Protocol

from bleak import BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic

from .protocol import encode_usb_sync_request

logger = logging.getLogger(__name__)

COUNTER_BITS = 32
TICKS_PER_SECOND = 1_000_000
NOMINAL_SCALE_NS = 1_000_000_000.0 / TICKS_PER_SECOND
MAX_ROUND_TRIP_NS = 250_000_000
FASTEST_OBSERVATION_COUNT = 5
MINIMUM_VALID_OBSERVATIONS = 3
DEFAULT_STARTUP_PINGS = 50
DEFAULT_REFRESH_PINGS = 10
DEFAULT_REFRESH_INTERVAL_S = 30.0
DEFAULT_PING_TIMEOUT_S = 1.0


@dataclass(frozen=True, slots=True)
class SyncObservation:
    request_id: int
    host_send_ns: int
    host_recv_ns: int
    device_ticks: int


class CounterUnwrapper:
    def __init__(self, counter_bits: int = COUNTER_BITS) -> None:
        self._counter_bits = counter_bits
        self._modulus = 1 << counter_bits
        self._latest: int | None = None

    def reset(self) -> None:
        self._latest = None

    def unwrap(self, value: int, advance: bool = True) -> int:
        raw = value & (self._modulus - 1)
        if self._latest is None:
            extended = raw
        else:
            epoch = round((self._latest - raw) / self._modulus)
            extended = raw + int(epoch) * self._modulus
            if advance and extended < self._latest:
                if (self._latest - extended) > (self._modulus // 2):
                    extended += self._modulus
                else:
                    extended = self._latest

        if advance and (self._latest is None or extended > self._latest):
            self._latest = extended
        return extended


def _median(values: list[float]) -> float:
    if not values:
        raise ValueError("median of empty list")
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


class DeviceClockMapper:
    """Map wrapping device micros() ticks to host monotonic time."""

    def __init__(self) -> None:
        self.calibrated = False
        self.offset_ns = 0.0
        self.scale = NOMINAL_SCALE_NS
        self._sample_unwrapper = CounterUnwrapper(COUNTER_BITS)
        self._sync_unwrapper = CounterUnwrapper(COUNTER_BITS)
        self._last_mapped_host_ns: int | None = None
        self._anchors: list[tuple[float, float]] = []

    def reset(self) -> None:
        self.calibrated = False
        self.offset_ns = 0.0
        self.scale = NOMINAL_SCALE_NS
        self._sample_unwrapper.reset()
        self._sync_unwrapper.reset()
        self._last_mapped_host_ns = None
        self._anchors.clear()

    def observe_burst(self, observations: list[SyncObservation], startup: bool = False) -> bool:
        valid: list[tuple[SyncObservation, int]] = []
        for observation in observations:
            round_trip_ns = observation.host_recv_ns - observation.host_send_ns
            if round_trip_ns < 0 or round_trip_ns > MAX_ROUND_TRIP_NS:
                continue
            valid.append((observation, round_trip_ns))

        if len(valid) < MINIMUM_VALID_OBSERVATIONS:
            logger.warning(
                "clock sync burst rejected: only %d/%d valid observations",
                len(valid),
                len(observations),
            )
            return False

        valid.sort(key=lambda item: item[1])
        valid = valid[:FASTEST_OBSERVATION_COUNT]
        valid.sort(key=lambda item: item[0].host_send_ns)

        device_values: list[float] = []
        offsets: list[float] = []
        for observation, _round_trip_ns in valid:
            extended = self._sync_unwrapper.unwrap(observation.device_ticks)
            device_ticks = float(extended)
            midpoint_ns = (observation.host_send_ns + observation.host_recv_ns) / 2.0
            device_values.append(device_ticks)
            offsets.append(midpoint_ns - NOMINAL_SCALE_NS * device_ticks)

        device_anchor = _median(device_values)
        offset_ns = _median(offsets)
        host_anchor = NOMINAL_SCALE_NS * device_anchor + offset_ns

        if startup or not self.calibrated:
            self._anchors = [(device_anchor, host_anchor)]
            self.scale = NOMINAL_SCALE_NS
            self.offset_ns = offset_ns
        else:
            self._anchors.append((device_anchor, host_anchor))
            if len(self._anchors) > 20:
                self._anchors = self._anchors[-20:]
            self._update_drift()

        self.calibrated = True
        self._last_mapped_host_ns = None
        phase = "startup" if startup else "refresh"
        drift_ppm = (self.scale / NOMINAL_SCALE_NS - 1.0) * 1_000_000.0
        logger.info(
            "clock sync %s: scale=%.6f ns/device_us (%.3f ppm) offset=%.3f ms (%.0f ns) anchors=%d",
            phase,
            self.scale,
            drift_ppm,
            self.offset_ns / 1_000_000.0,
            self.offset_ns,
            len(self._anchors),
        )
        return True

    def map_device_us(self, device_ticks: int) -> int | None:
        if not self.calibrated:
            return None

        extended = self._sample_unwrapper.unwrap(device_ticks)
        host_ns = self.scale * float(extended) + self.offset_ns
        if not float("inf") > host_ns > float("-inf"):
            return None

        mapped_ns = int(round(host_ns))
        if self._last_mapped_host_ns is not None and mapped_ns < self._last_mapped_host_ns:
            mapped_ns = self._last_mapped_host_ns
        self._last_mapped_host_ns = mapped_ns
        return mapped_ns // 1_000

    def _update_drift(self) -> None:
        if len(self._anchors) < 2:
            return

        device_mean = sum(point[0] for point in self._anchors) / len(self._anchors)
        host_mean = sum(point[1] for point in self._anchors) / len(self._anchors)

        numerator = 0.0
        denominator = 0.0
        for device_ticks, host_ns in self._anchors:
            centered_device = device_ticks - device_mean
            numerator += centered_device * (host_ns - host_mean)
            denominator += centered_device * centered_device

        if denominator <= 0.0:
            return

        fitted_scale = numerator / denominator
        fitted_offset = host_mean - fitted_scale * device_mean
        if fitted_scale <= 0.0:
            return

        self.scale = fitted_scale
        self.offset_ns = fitted_offset


class _SyncSend(Protocol):
    async def __call__(self, request_id: int) -> None: ...


class SyncSession:
    def __init__(
        self,
        mapper: DeviceClockMapper,
        ping_timeout_s: float = DEFAULT_PING_TIMEOUT_S,
    ) -> None:
        self._mapper = mapper
        self._ping_timeout_s = ping_timeout_s
        self._request_id = 0
        self._pending: dict[int, tuple[int, asyncio.Future[SyncObservation]]] = {}
        self._refresh_task: asyncio.Task[None] | None = None
        self._stop_refresh = asyncio.Event()

    @property
    def mapper(self) -> DeviceClockMapper:
        return self._mapper

    def handle_sync_response(self, request_id: int, device_ticks: int) -> None:
        pending = self._pending.pop(request_id, None)
        if pending is None:
            return

        send_ns, future = pending
        observation = SyncObservation(
            request_id=request_id,
            host_send_ns=send_ns,
            host_recv_ns=time.monotonic_ns(),
            device_ticks=device_ticks,
        )
        if not future.done():
            future.set_result(observation)

    async def ping(self, send_request: _SyncSend) -> SyncObservation | None:
        self._request_id = (self._request_id + 1) & 0xFFFFFFFF
        request_id = self._request_id
        loop = asyncio.get_running_loop()
        future: asyncio.Future[SyncObservation] = loop.create_future()
        send_ns = time.monotonic_ns()
        self._pending[request_id] = (send_ns, future)

        try:
            await send_request(request_id)
        except Exception:
            self._pending.pop(request_id, None)
            logger.exception("clock sync request failed for id %d", request_id)
            return None

        try:
            return await asyncio.wait_for(future, timeout=self._ping_timeout_s)
        except asyncio.TimeoutError:
            self._pending.pop(request_id, None)
            return None

    async def run_burst(self, count: int, send_request: _SyncSend, startup: bool = False) -> bool:
        observations: list[SyncObservation] = []
        for _ in range(count):
            observation = await self.ping(send_request)
            if observation is not None:
                observations.append(observation)
        return self._mapper.observe_burst(observations, startup=startup)

    async def run_startup(self, count: int, send_request: _SyncSend) -> bool:
        self._mapper.reset()
        return await SyncSession.run_burst(self, count, send_request, startup=True)

    async def start_periodic_refresh(
        self,
        send_request: _SyncSend,
        ping_count: int = DEFAULT_REFRESH_PINGS,
        interval_s: float = DEFAULT_REFRESH_INTERVAL_S,
    ) -> None:
        self._stop_refresh.clear()

        async def refresh_loop() -> None:
            while not self._stop_refresh.is_set():
                try:
                    await asyncio.wait_for(self._stop_refresh.wait(), timeout=interval_s)
                    break
                except asyncio.TimeoutError:
                    if not self._mapper.calibrated:
                        continue
                    if not await SyncSession.run_burst(
                        self, ping_count, send_request, startup=False
                    ):
                        logger.warning("periodic clock sync refresh failed")

        self._refresh_task = asyncio.create_task(refresh_loop(), name="clock-sync-refresh")

    async def stop_periodic_refresh(self) -> None:
        self._stop_refresh.set()
        if self._refresh_task is None:
            return
        self._refresh_task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await self._refresh_task
        self._refresh_task = None


class BleClockSyncClient(SyncSession):
    def __init__(
        self,
        client: BleakClient,
        sync_char: BleakGATTCharacteristic,
        mapper: DeviceClockMapper,
        ping_timeout_s: float = DEFAULT_PING_TIMEOUT_S,
    ) -> None:
        super().__init__(mapper, ping_timeout_s=ping_timeout_s)
        self._client = client
        self._sync_char = sync_char

    def on_sync_notification(self, data: bytes) -> None:
        if len(data) != 8:
            return
        request_id, device_ticks = struct.unpack("<II", data)
        self.handle_sync_response(request_id, device_ticks)

    async def _send_ble(self, request_id: int) -> None:
        await self._client.write_gatt_char(
            self._sync_char,
            struct.pack("<I", request_id),
            response=True,
        )

    async def run_startup(self, ping_count: int = DEFAULT_STARTUP_PINGS) -> bool:
        return await SyncSession.run_startup(self, ping_count, self._send_ble)

    async def start_periodic_refresh(
        self,
        ping_count: int = DEFAULT_REFRESH_PINGS,
        interval_s: float = DEFAULT_REFRESH_INTERVAL_S,
    ) -> None:
        await SyncSession.start_periodic_refresh(
            self,
            self._send_ble,
            ping_count=ping_count,
            interval_s=interval_s,
        )


class UsbClockSyncClient(SyncSession):
    def __init__(
        self,
        mapper: DeviceClockMapper,
        write_frame: Callable[[bytes], Awaitable[None]],
        ping_timeout_s: float = DEFAULT_PING_TIMEOUT_S,
    ) -> None:
        super().__init__(mapper, ping_timeout_s=ping_timeout_s)
        self._write_frame = write_frame

    def on_sync_response_message(self, payload: bytes) -> None:
        if len(payload) != 8:
            return
        request_id, device_ticks = struct.unpack("<II", payload)
        self.handle_sync_response(request_id, device_ticks)

    async def _send_usb(self, request_id: int) -> None:
        await self._write_frame(encode_usb_sync_request(request_id))

    async def run_startup(self, ping_count: int = DEFAULT_STARTUP_PINGS) -> bool:
        return await SyncSession.run_startup(self, ping_count, self._send_usb)

    async def start_periodic_refresh(
        self,
        ping_count: int = DEFAULT_REFRESH_PINGS,
        interval_s: float = DEFAULT_REFRESH_INTERVAL_S,
    ) -> None:
        await SyncSession.start_periodic_refresh(
            self,
            self._send_usb,
            ping_count=ping_count,
            interval_s=interval_s,
        )


# Backward-compatible alias used by the BLE client.
ClockSyncClient = BleClockSyncClient
