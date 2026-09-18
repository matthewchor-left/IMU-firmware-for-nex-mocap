"""USB CDC client for the XiaoIMU serial stream."""

from __future__ import annotations

import asyncio
import contextlib
import logging
from collections.abc import Awaitable, Callable

import serial
from serial.serialutil import SerialException

from .clock_sync import (
    DEFAULT_REFRESH_INTERVAL_S,
    DEFAULT_REFRESH_PINGS,
    DEFAULT_STARTUP_PINGS,
    DeviceClockMapper,
    UsbClockSyncClient,
)
from .protocol import (
    USB_MSG_IMU_BATCH,
    USB_MSG_SYNC_RESP,
    VERSION,
    VERSION_ALIGNED,
    UsbFrameStream,
    UsbMessage,
    build_aligned_payload,
    parse_samples,
)

logger = logging.getLogger(__name__)

OnBatchCallback = Callable[[bytes, int], Awaitable[None] | None]
OnDeviceBatchCallback = Callable[[bytes], Awaitable[None] | None]
OnSessionCallback = Callable[[], Awaitable[None] | None]


class UsbImuClient:
    def __init__(
        self,
        *,
        port: str,
        baudrate: int = 115200,
        on_batch: OnBatchCallback,
        on_device_batch: OnDeviceBatchCallback | None = None,
        on_session_start: OnSessionCallback | None = None,
        read_size: int = 4096,
        reconnect_delay_s: float = 1.0,
        align_timestamps: bool = True,
        startup_sync_pings: int = DEFAULT_STARTUP_PINGS,
        refresh_sync_pings: int = DEFAULT_REFRESH_PINGS,
        refresh_interval_s: float = DEFAULT_REFRESH_INTERVAL_S,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.on_batch = on_batch
        self.on_device_batch = on_device_batch
        self.on_session_start = on_session_start
        self.read_size = read_size
        self.reconnect_delay_s = reconnect_delay_s
        self.align_timestamps = align_timestamps
        self.startup_sync_pings = startup_sync_pings
        self.refresh_sync_pings = refresh_sync_pings
        self.refresh_interval_s = refresh_interval_s
        self._clock_sync: UsbClockSyncClient | None = None
        self._streaming = False
        self._stop = asyncio.Event()

    @property
    def clock_mapper(self) -> DeviceClockMapper | None:
        if self._clock_sync is None:
            return None
        return self._clock_sync.mapper

    async def run(self) -> None:
        backoff_s = self.reconnect_delay_s

        while not self._stop.is_set():
            try:
                logger.info("opening USB serial port %s", self.port)
                await self._read_loop()
                backoff_s = self.reconnect_delay_s
            except asyncio.CancelledError:
                raise
            except SerialException:
                logger.exception("USB serial session ended")
            except Exception:
                logger.exception("USB reader failed")
            finally:
                self._streaming = False
                if self._clock_sync is not None:
                    await self._clock_sync.stop_periodic_refresh()
                    self._clock_sync = None

            if self._stop.is_set():
                break

            logger.info("reconnecting USB in %.1f s", backoff_s)
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=backoff_s)
                break
            except asyncio.TimeoutError:
                pass

            backoff_s = min(backoff_s * 2.0, 30.0)

    async def stop(self) -> None:
        self._stop.set()

    async def _process_messages(
        self,
        messages: list[UsbMessage],
        clock_sync: UsbClockSyncClient,
    ) -> None:
        for message in messages:
            if message.msg_type == USB_MSG_SYNC_RESP:
                clock_sync.on_sync_response_message(message.payload)
                continue

            if message.msg_type != USB_MSG_IMU_BATCH:
                logger.debug(
                    "ignoring USB message type 0x%02x (%d bytes)",
                    message.msg_type,
                    len(message.payload),
                )
                continue

            if not self._streaming:
                continue

            await self._dispatch_batch(message.payload)

    async def _reader_loop(
        self,
        ser: serial.Serial,
        stream: UsbFrameStream,
        clock_sync: UsbClockSyncClient,
    ) -> None:
        while not self._stop.is_set():
            chunk = await asyncio.to_thread(ser.read, self.read_size)
            if not chunk:
                await asyncio.sleep(0.001)
                continue

            await self._process_messages(stream.feed(chunk), clock_sync)

    async def _read_loop(self) -> None:
        stream = UsbFrameStream()

        with serial.Serial(
            self.port,
            baudrate=self.baudrate,
            timeout=0.1,
        ) as ser:
            logger.info("USB serial connected (%s @ %d)", self.port, self.baudrate)

            async def write_frame(frame: bytes) -> None:
                await asyncio.to_thread(ser.write, frame)
                await asyncio.to_thread(ser.flush)

            mapper = DeviceClockMapper()
            clock_sync = UsbClockSyncClient(mapper, write_frame)
            self._clock_sync = clock_sync

            reader_task = asyncio.create_task(
                self._reader_loop(ser, stream, clock_sync),
                name="usb-reader",
            )

            try:
                if self.on_session_start is not None:
                    result = self.on_session_start()
                    if asyncio.iscoroutine(result):
                        await result

                if self.align_timestamps:
                    logger.info(
                        "running USB startup clock sync (%d pings)",
                        self.startup_sync_pings,
                    )
                    if not await clock_sync.run_startup(self.startup_sync_pings):
                        raise RuntimeError("USB startup clock sync failed")
                    await clock_sync.start_periodic_refresh(
                        ping_count=self.refresh_sync_pings,
                        interval_s=self.refresh_interval_s,
                    )
                    logger.info("USB clock sync ready; streaming host-aligned timestamps")
                else:
                    logger.info("using raw device timestamps over USB (protocol v1)")

                self._streaming = True
                await reader_task
            finally:
                reader_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await reader_task

    async def _dispatch_batch(self, payload: bytes) -> None:
        try:
            samples = parse_samples(payload)
        except ValueError:
            logger.warning("ignoring malformed USB IMU batch (%d bytes)", len(payload))
            return

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(
                "received %d USB sample(s), seq %d..%d",
                len(samples),
                samples[0].sequence,
                samples[-1].sequence,
            )

        if self.on_device_batch is not None:
            result = self.on_device_batch(payload)
            if asyncio.iscoroutine(result):
                await result

        version = VERSION
        if self.align_timestamps:
            mapper = self.clock_mapper
            if mapper is None or not mapper.calibrated:
                return
            try:
                payload = build_aligned_payload(payload, mapper)
            except RuntimeError:
                logger.warning("dropping USB batch because clock mapper is not calibrated")
                return
            version = VERSION_ALIGNED

        result = self.on_batch(payload, version)
        if asyncio.iscoroutine(result):
            await result
