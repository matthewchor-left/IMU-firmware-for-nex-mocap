"""BLE central client for the XiaoIMU peripheral."""

from __future__ import annotations

import asyncio
import contextlib
import logging
from collections.abc import Awaitable, Callable

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

from .clock_sync import (
    DEFAULT_REFRESH_INTERVAL_S,
    DEFAULT_REFRESH_PINGS,
    DEFAULT_STARTUP_PINGS,
    ClockSyncClient,
    DeviceClockMapper,
)
from .protocol import (
    DEFAULT_DEVICE_NAME,
    IMU_CHAR_UUID,
    SERVICE_UUID,
    SYNC_CHAR_UUID,
    VERSION,
    VERSION_ALIGNED,
    build_aligned_payload,
    parse_samples,
)
from .stats import batch_capacity_for_mtu

logger = logging.getLogger(__name__)

OnBatchCallback = Callable[[bytes, int], Awaitable[None] | None]
OnSessionCallback = Callable[[int], Awaitable[None] | None]


async def negotiate_mtu(client: BleakClient) -> int:
    backend = client._backend
    acquire_mtu = getattr(backend, "_acquire_mtu", None)
    if acquire_mtu is not None:
        await acquire_mtu()
    return client.mtu_size


class XiaoBleClient:
    def __init__(
        self,
        *,
        device_name: str = DEFAULT_DEVICE_NAME,
        address: str | None = None,
        on_batch: OnBatchCallback,
        on_session_start: OnSessionCallback | None = None,
        scan_timeout_s: float = 10.0,
        align_timestamps: bool = True,
        startup_sync_pings: int = DEFAULT_STARTUP_PINGS,
        refresh_sync_pings: int = DEFAULT_REFRESH_PINGS,
        refresh_interval_s: float = DEFAULT_REFRESH_INTERVAL_S,
    ) -> None:
        self.device_name = device_name
        self.address = address
        self.on_batch = on_batch
        self.on_session_start = on_session_start
        self.scan_timeout_s = scan_timeout_s
        self.align_timestamps = align_timestamps
        self.startup_sync_pings = startup_sync_pings
        self.refresh_sync_pings = refresh_sync_pings
        self.refresh_interval_s = refresh_interval_s
        self._client: BleakClient | None = None
        self._clock_sync: ClockSyncClient | None = None
        self._streaming = False
        self._stop = asyncio.Event()

    @property
    def clock_mapper(self) -> DeviceClockMapper | None:
        if self._clock_sync is None:
            return None
        return self._clock_sync.mapper

    async def run(self) -> None:
        backoff_s = 1.0

        while not self._stop.is_set():
            try:
                device = await self._find_device()
                logger.info("connecting to %s (%s)", device.name or "unknown", device.address)
                await self._stream_device(device)
                backoff_s = 1.0
            except asyncio.CancelledError:
                raise
            except Exception:
                logger.exception("BLE session ended")
            finally:
                await self._disconnect()

            if self._stop.is_set():
                break

            logger.info("reconnecting in %.1f s", backoff_s)
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=backoff_s)
                break
            except asyncio.TimeoutError:
                pass

            backoff_s = min(backoff_s * 2.0, 30.0)

    async def stop(self) -> None:
        self._stop.set()
        await self._disconnect()

    async def _find_device(self) -> BLEDevice:
        if self.address:
            device = await BleakScanner.find_device_by_address(
                self.address,
                timeout=self.scan_timeout_s,
            )
            if device is None:
                raise RuntimeError(f"device not found at address {self.address}")
            return device

        def match(device: BLEDevice, advertisement: AdvertisementData) -> bool:
            if device.name == self.device_name:
                return True
            service_uuids = {uuid.lower() for uuid in advertisement.service_uuids}
            return SERVICE_UUID.lower() in service_uuids

        device = await BleakScanner.find_device_by_filter(
            match,
            timeout=self.scan_timeout_s,
        )
        if device is None:
            raise RuntimeError(
                f"device '{self.device_name}' not found within {self.scan_timeout_s:.0f} s"
            )
        return device

    async def _stream_device(self, device: BLEDevice) -> None:
        loop = asyncio.get_running_loop()
        disconnected = asyncio.Event()
        self._streaming = False
        self._clock_sync = None

        def on_disconnect(_client: BleakClient) -> None:
            logger.info("BLE disconnected")
            loop.call_soon_threadsafe(disconnected.set)

        async with BleakClient(device, disconnected_callback=on_disconnect) as client:
            self._client = client
            services = client.services
            imu_service = services.get_service(SERVICE_UUID)
            if imu_service is None:
                raise RuntimeError(f"service {SERVICE_UUID} not found")

            imu_char = imu_service.get_characteristic(IMU_CHAR_UUID)
            sync_char = imu_service.get_characteristic(SYNC_CHAR_UUID)
            if imu_char is None:
                raise RuntimeError(f"characteristic {IMU_CHAR_UUID} not found")
            if sync_char is None:
                raise RuntimeError(f"characteristic {SYNC_CHAR_UUID} not found")

            mtu = await negotiate_mtu(client)
            batch_capacity = batch_capacity_for_mtu(mtu)
            if batch_capacity == 0:
                logger.warning(
                    "negotiated MTU %d is too small for IMU packets (need >= 32); "
                    "device may not stream data",
                    mtu,
                )
            else:
                logger.info(
                    "negotiated MTU %d (up to %d sample(s) per BLE notification)",
                    mtu,
                    batch_capacity,
                )

            if self.on_session_start is not None:
                result = self.on_session_start(mtu)
                if asyncio.iscoroutine(result):
                    await result

            mapper = DeviceClockMapper()
            clock_sync = ClockSyncClient(client, sync_char, mapper)
            self._clock_sync = clock_sync

            def on_sync(_handle: int, data: bytearray) -> None:
                clock_sync.on_sync_notification(bytes(data))

            def on_imu(_handle: int, data: bytearray) -> None:
                if not self._streaming:
                    return
                payload = bytes(data)
                loop.create_task(self._dispatch_batch(payload))

            await client.start_notify(sync_char, on_sync)
            await client.start_notify(imu_char, on_imu)
            logger.info("BLE notifications enabled")

            if self.align_timestamps:
                logger.info("running startup clock sync (%d pings)", self.startup_sync_pings)
                if not await clock_sync.run_startup(self.startup_sync_pings):
                    raise RuntimeError("startup clock sync failed")
                await clock_sync.start_periodic_refresh(
                    ping_count=self.refresh_sync_pings,
                    interval_s=self.refresh_interval_s,
                )
                logger.info("clock sync ready; starting IMU stream with host-aligned timestamps")
            else:
                logger.info("using raw device timestamps (protocol v1)")

            self._streaming = True
            if batch_capacity == 0:
                logger.warning(
                    "waiting for IMU stream, but firmware will not transmit until MTU is large enough"
                )

            stop_task = asyncio.create_task(self._stop.wait())
            disconnect_task = asyncio.create_task(disconnected.wait())
            _, pending = await asyncio.wait(
                {stop_task, disconnect_task},
                return_when=asyncio.FIRST_COMPLETED,
            )
            for task in pending:
                task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await asyncio.gather(*pending)

    async def _dispatch_batch(self, payload: bytes) -> None:
        try:
            samples = parse_samples(payload)
        except ValueError:
            logger.warning("ignoring malformed IMU notification (%d bytes)", len(payload))
            return

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(
                "received %d sample(s), seq %d..%d",
                len(samples),
                samples[0].sequence,
                samples[-1].sequence,
            )

        version = VERSION
        if self.align_timestamps:
            mapper = self.clock_mapper
            if mapper is None or not mapper.calibrated:
                return
            try:
                payload = build_aligned_payload(payload, mapper)
            except RuntimeError:
                logger.warning("dropping batch because clock mapper is not calibrated")
                return
            version = VERSION_ALIGNED

        result = self.on_batch(payload, version)
        if asyncio.iscoroutine(result):
            await result

    async def _disconnect(self) -> None:
        self._streaming = False
        if self._clock_sync is not None:
            await self._clock_sync.stop_periodic_refresh()
            self._clock_sync = None

        client = self._client
        self._client = None
        if client is None or not client.is_connected:
            return

        try:
            await client.disconnect()
        except Exception:
            logger.exception("error while disconnecting BLE client")
