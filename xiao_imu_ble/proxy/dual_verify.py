"""Compare BLE and USB clock sync and sample streams in dual-transport mode."""

from __future__ import annotations

import logging
import statistics
from dataclasses import dataclass, field

from .clock_sync import NOMINAL_SCALE_NS, DeviceClockMapper
from .protocol import ImuSample, parse_samples

logger = logging.getLogger(__name__)


def format_clock_comparison(ble_mapper: DeviceClockMapper, usb_mapper: DeviceClockMapper) -> str | None:
    if not ble_mapper.calibrated or not usb_mapper.calibrated:
        return None

    ble_ppm = (ble_mapper.scale / NOMINAL_SCALE_NS - 1.0) * 1_000_000.0
    usb_ppm = (usb_mapper.scale / NOMINAL_SCALE_NS - 1.0) * 1_000_000.0
    d_offset_ms = (ble_mapper.offset_ns - usb_mapper.offset_ns) / 1_000_000.0
    d_ppm = ble_ppm - usb_ppm

    return (
        "sync compare: "
        f"ble offset={ble_mapper.offset_ns / 1_000_000.0:.3f} ms "
        f"scale={ble_ppm:.3f} ppm | "
        f"usb offset={usb_mapper.offset_ns / 1_000_000.0:.3f} ms "
        f"scale={usb_ppm:.3f} ppm | "
        f"delta offset={d_offset_ms:.3f} ms scale={d_ppm:.3f} ppm"
    )


def log_clock_comparison(ble_mapper: DeviceClockMapper, usb_mapper: DeviceClockMapper) -> None:
    summary = format_clock_comparison(ble_mapper, usb_mapper)
    if summary is not None:
        logger.info(summary)


@dataclass
class DualVerifyStats:
    matched_pairs: int = 0
    axis_mismatches: int = 0
    device_ts_mismatches: int = 0
    host_ts_deltas_us: list[int] = field(default_factory=list)
    max_axis_delta: float = 0.0

    def format_summary(self) -> str:
        if not self.host_ts_deltas_us:
            host_ts_summary = "host_ts_delta: n/a"
        else:
            abs_deltas = [abs(value) for value in self.host_ts_deltas_us[-256:]]
            host_ts_summary = (
                f"host_ts_delta median={statistics.median(abs_deltas):.1f} us "
                f"max={max(abs_deltas):.1f} us (n={len(self.host_ts_deltas_us)})"
            )

        return (
            f"verify matched={self.matched_pairs} "
            f"axis_mismatch={self.axis_mismatches} "
            f"device_ts_mismatch={self.device_ts_mismatches} "
            f"max_axis_delta={self.max_axis_delta:.6f} "
            f"{host_ts_summary}"
        )


class DualVerifier:
    def __init__(self, axis_tolerance: float = 1e-3) -> None:
        self.axis_tolerance = axis_tolerance
        self.stats = DualVerifyStats()
        self._ble_samples: dict[int, ImuSample] = {}
        self._usb_samples: dict[int, ImuSample] = {}
        self._ble_host_ts: dict[int, int] = {}
        self._usb_host_ts: dict[int, int] = {}

    def ingest_ble(
        self,
        payload: bytes,
        mapper: DeviceClockMapper | None,
    ) -> None:
        self._ingest(payload, mapper, self._ble_samples, self._ble_host_ts, "ble")

    def ingest_usb(
        self,
        payload: bytes,
        mapper: DeviceClockMapper | None,
    ) -> None:
        self._ingest(payload, mapper, self._usb_samples, self._usb_host_ts, "usb")

    def _ingest(
        self,
        payload: bytes,
        mapper: DeviceClockMapper | None,
        sample_store: dict[int, ImuSample],
        host_store: dict[int, int],
        label: str,
    ) -> None:
        for sample in parse_samples(payload):
            sample_store[sample.sequence] = sample
            if mapper is not None and mapper.calibrated:
                host_ts = mapper.map_device_us(sample.timestamp_us)
                if host_ts is not None:
                    host_store[sample.sequence] = host_ts
            self._try_match(sample.sequence, label)

    def _try_match(self, sequence: int, ingested_via: str) -> None:
        if sequence not in self._ble_samples or sequence not in self._usb_samples:
            return

        ble = self._ble_samples.pop(sequence)
        usb = self._usb_samples.pop(sequence)
        self.stats.matched_pairs += 1

        if ble.timestamp_us != usb.timestamp_us:
            self.stats.device_ts_mismatches += 1
            logger.warning(
                "sequence %d device timestamp mismatch: ble=%d usb=%d",
                sequence,
                ble.timestamp_us,
                usb.timestamp_us,
            )

        axis_delta = max(
            abs(ble.gx - usb.gx),
            abs(ble.gy - usb.gy),
            abs(ble.gz - usb.gz),
            abs(ble.ax - usb.ax),
            abs(ble.ay - usb.ay),
            abs(ble.az - usb.az),
        )
        self.stats.max_axis_delta = max(self.stats.max_axis_delta, axis_delta)
        if axis_delta > self.axis_tolerance:
            self.stats.axis_mismatches += 1
            logger.warning(
                "sequence %d axis mismatch via %s: max_delta=%.6f",
                sequence,
                ingested_via,
                axis_delta,
            )

        ble_host_ts = self._ble_host_ts.pop(sequence, None)
        usb_host_ts = self._usb_host_ts.pop(sequence, None)
        if ble_host_ts is not None and usb_host_ts is not None:
            self.stats.host_ts_deltas_us.append(ble_host_ts - usb_host_ts)
