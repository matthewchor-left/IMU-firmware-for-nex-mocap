"""Run BLE and USB transports together for clock-sync verification."""

from __future__ import annotations

import asyncio
import contextlib
import logging
from collections.abc import Awaitable, Callable

from .ble_client import XiaoBleClient
from .clock_sync import DEFAULT_REFRESH_INTERVAL_S, DEFAULT_REFRESH_PINGS, DEFAULT_STARTUP_PINGS
from .dual_verify import DualVerifier, log_clock_comparison
from .protocol import parse_samples
from .stats import ProxyStats
from .usb_client import UsbImuClient

logger = logging.getLogger(__name__)

OnBatchCallback = Callable[[bytes], Awaitable[None] | None]


class DualImuClient:
    def __init__(
        self,
        *,
        ble_device_name: str,
        ble_address: str | None,
        usb_port: str,
        usb_baudrate: int,
        on_batch: OnBatchCallback,
        tcp_source: str = "ble",
        ble_stats: ProxyStats,
        usb_stats: ProxyStats,
        scan_timeout_s: float = 10.0,
        startup_sync_pings: int = DEFAULT_STARTUP_PINGS,
        refresh_sync_pings: int = DEFAULT_REFRESH_PINGS,
        refresh_interval_s: float = DEFAULT_REFRESH_INTERVAL_S,
        compare_interval_s: float = 5.0,
        axis_tolerance: float = 1e-3,
    ) -> None:
        self.tcp_source = tcp_source
        self.ble_stats = ble_stats
        self.usb_stats = usb_stats
        self.compare_interval_s = compare_interval_s
        self._verifier = DualVerifier(axis_tolerance=axis_tolerance)
        self._on_batch = on_batch
        self._stop = asyncio.Event()
        self._firmware_hint_logged = False

        self._ble_client = XiaoBleClient(
            device_name=ble_device_name,
            address=ble_address,
            on_batch=self._on_ble_batch,
            on_device_batch=self._on_ble_device_batch,
            on_session_start=self._on_ble_session_start,
            scan_timeout_s=scan_timeout_s,
            startup_sync_pings=startup_sync_pings,
            refresh_sync_pings=refresh_sync_pings,
            refresh_interval_s=refresh_interval_s,
        )
        self._usb_client = UsbImuClient(
            port=usb_port,
            baudrate=usb_baudrate,
            on_batch=self._on_usb_batch,
            on_device_batch=self._on_usb_device_batch,
            on_session_start=self._on_usb_session_start,
            startup_sync_pings=startup_sync_pings,
            refresh_sync_pings=refresh_sync_pings,
            refresh_interval_s=refresh_interval_s,
        )

    async def run(self) -> None:
        compare_task = asyncio.create_task(self._compare_loop(), name="dual-compare")
        ble_task = asyncio.create_task(self._ble_client.run(), name="dual-ble")
        usb_task = asyncio.create_task(self._usb_client.run(), name="dual-usb")

        try:
            await asyncio.wait(
                {ble_task, usb_task},
                return_when=asyncio.FIRST_COMPLETED,
            )
        finally:
            self._stop.set()
            await self._ble_client.stop()
            await self._usb_client.stop()
            compare_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await asyncio.gather(ble_task, usb_task, compare_task, return_exceptions=True)

    async def stop(self) -> None:
        self._stop.set()
        await self._ble_client.stop()
        await self._usb_client.stop()

    def _on_ble_session_start(self, mtu: int) -> None:
        self.ble_stats.reset_session(mtu)

    def _on_usb_session_start(self) -> None:
        self.usb_stats.reset_session(0)

    async def _on_ble_device_batch(self, payload: bytes) -> None:
        self.ble_stats.record_ble_batch(parse_samples(payload))
        self._verifier.ingest_ble(payload, self._ble_client.clock_mapper)

    async def _on_usb_device_batch(self, payload: bytes) -> None:
        self.usb_stats.record_ble_batch(parse_samples(payload))
        self._verifier.ingest_usb(payload, self._usb_client.clock_mapper)

    async def _on_ble_batch(self, payload: bytes) -> None:
        if self.tcp_source == "ble":
            await self._on_batch(payload)

    async def _on_usb_batch(self, payload: bytes) -> None:
        if self.tcp_source == "usb":
            await self._on_batch(payload)

    def _maybe_log_firmware_hint(self) -> None:
        if self._firmware_hint_logged:
            return
        if self.usb_stats.ble_samples <= 0 or self.ble_stats.ble_samples > 0:
            return

        self._firmware_hint_logged = True
        logger.warning(
            "USB is streaming but BLE has not received any samples "
            "(check that BLE is advertising as '%s')",
            self._ble_client.device_name,
        )

    async def _compare_loop(self) -> None:
        while not self._stop.is_set():
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=self.compare_interval_s)
                break
            except asyncio.TimeoutError:
                pass

            ble_mapper = self._ble_client.clock_mapper
            usb_mapper = self._usb_client.clock_mapper
            if ble_mapper is not None and usb_mapper is not None:
                log_clock_comparison(ble_mapper, usb_mapper)

            self._maybe_log_firmware_hint()

            logger.info("dual %s", self._verifier.stats.format_summary())
            logger.info("dual ble %s", self.ble_stats.format_summary(transport="ble"))
            logger.info("dual usb %s", self.usb_stats.format_summary(transport="usb"))
