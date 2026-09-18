"""Bridge XiaoIMU transports to a localhost TCP stream."""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import logging
import sys
import time
from typing import Protocol, TextIO

from .clock_sync import DEFAULT_REFRESH_INTERVAL_S, DEFAULT_REFRESH_PINGS, DEFAULT_STARTUP_PINGS
from .protocol import (
    DEFAULT_DEVICE_NAME,
    ImuSample,
    parse_aligned_samples,
)
from .mock_client import DEFAULT_MOCK_BATCH_SIZE, DEFAULT_MOCK_RATE_HZ, MockImuClient
from .rebatch import TcpRebatcher
from .stats import ProxyStats

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
DEFAULT_SERIAL_PORT = "auto"
QUEUE_CAPACITY = 256
DEFAULT_STATS_INTERVAL_S = 5.0
DEFAULT_IDLE_WARN_S = 3.0
DEFAULT_REBATCH_SAMPLES = 0
DEFAULT_REBATCH_MS = 50.0


class InputClient(Protocol):
    async def run(self) -> None: ...

    async def stop(self) -> None: ...


class TcpBroadcaster:
    def __init__(
        self,
        host: str,
        port: int,
        stats: ProxyStats,
        *,
        rebatch_samples: int = DEFAULT_REBATCH_SAMPLES,
        rebatch_ms: float = DEFAULT_REBATCH_MS,
    ) -> None:
        self.host = host
        self.port = port
        self.stats = stats
        self._rebatcher = TcpRebatcher(rebatch_samples, rebatch_ms / 1000.0)
        self._queue: asyncio.Queue[bytes] = asyncio.Queue(maxsize=QUEUE_CAPACITY)
        self._server: asyncio.Server | None = None
        self._client_writer: asyncio.StreamWriter | None = None
        self._client_addr: tuple[str, int] | None = None

    async def start(self) -> None:
        self._server = await asyncio.start_server(
            self._handle_client,
            self.host,
            self.port,
        )
        sockets = ", ".join(str(sock.getsockname()) for sock in self._server.sockets or ())
        logging.info("TCP server listening on %s", sockets)

    async def stop(self) -> None:
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
            self._server = None

        await self._close_client("server stopping")

    def reset_session(self, mtu: int = 0) -> None:
        self.stats.reset_session(mtu)

    async def publish_batch(self, payload: bytes) -> None:
        samples = [
            ImuSample(
                sample.host_timestamp_us,
                sample.sequence,
                sample.gx,
                sample.gy,
                sample.gz,
                sample.ax,
                sample.ay,
                sample.az,
            )
            for sample in parse_aligned_samples(payload)
        ]
        self.stats.record_ble_batch(samples)

        for frame in self._rebatcher.ingest(payload):
            self._enqueue_frame(frame)

    async def flush_rebatch(self) -> None:
        for frame in self._rebatcher.flush():
            self._enqueue_frame(frame)

    def _enqueue_frame(self, frame: bytes) -> None:
        try:
            self._queue.put_nowait(frame)
        except asyncio.QueueFull:
            try:
                self._queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
            self.stats.record_tcp_drop()
            self._queue.put_nowait(frame)
            dropped = self.stats.dropped_tcp_frames
            if dropped == 1 or dropped % 50 == 0:
                logging.warning(
                    "TCP queue overflow; dropped %d frame(s) total",
                    dropped,
                )

    async def run_sender(self, stop_event: asyncio.Event) -> None:
        while not stop_event.is_set():
            try:
                frame = await asyncio.wait_for(self._queue.get(), timeout=0.5)
            except asyncio.TimeoutError:
                for stale_frame in self._rebatcher.flush_if_stale():
                    self._enqueue_frame(stale_frame)
                continue

            writer = self._client_writer
            if writer is None:
                continue

            try:
                writer.write(frame)
                await writer.drain()
                self.stats.record_tcp_send(len(frame))
            except ConnectionError:
                await self._close_client("client disconnected during send")
            except Exception:
                logging.exception("failed to send frame to TCP client")
                await self._close_client("send failure")

    async def _handle_client(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        peer = writer.get_extra_info("peername")
        if self._client_writer is not None:
            logging.info("replacing existing TCP client %s with %s", self._client_addr, peer)
            await self._close_client("replaced by new client")

        self._client_writer = writer
        self._client_addr = peer
        logging.info("TCP client connected from %s", peer)

        try:
            while True:
                data = await reader.read(1024)
                if not data:
                    break
        except ConnectionError:
            pass
        finally:
            if self._client_writer is writer:
                await self._close_client("client closed connection")

    async def _close_client(self, reason: str) -> None:
        writer = self._client_writer
        if writer is None:
            return

        self._client_writer = None
        self._client_addr = None
        logging.info("TCP client disconnected (%s)", reason)

        writer.close()
        try:
            await writer.wait_closed()
        except Exception:
            pass


async def run_stats_reporter(
    stats: ProxyStats,
    interval_s: float,
    idle_warn_s: float,
    transport: str,
    stop_event: asyncio.Event,
) -> None:
    while not stop_event.is_set():
        try:
            await asyncio.wait_for(stop_event.wait(), timeout=interval_s)
            break
        except asyncio.TimeoutError:
            pass

        logging.info("stats: %s", stats.format_summary(transport=transport))

        idle_for = stats.seconds_since_last_batch()
        if idle_for is not None and idle_for >= idle_warn_s:
            if transport == "usb":
                logging.warning(
                    "no IMU batches received for %.1f s (check USB cable and serial port)",
                    idle_for,
                )
            elif transport == "mock":
                logging.warning("no mock IMU batches generated for %.1f s", idle_for)
            else:
                logging.warning(
                    "no IMU batches received for %.1f s (check BLE link and MTU)",
                    idle_for,
                )
        elif idle_for is None and (time.monotonic() - stats.session_started_at) >= idle_warn_s:
            if transport == "usb":
                logging.warning(
                    "no IMU batches received since USB session started (check serial port)"
                )
            elif transport == "mock":
                logging.warning("no mock IMU batches generated since session started")
            else:
                logging.warning(
                    "no IMU batches received since BLE session started (check MTU and device link)"
                )


async def run_proxy(args: argparse.Namespace) -> None:
    stop_event = asyncio.Event()
    tcp_stats = ProxyStats()
    ble_stats = ProxyStats()
    usb_stats = ProxyStats()
    if args.mock_rate <= 0:
        raise SystemExit("--mock-rate must be greater than zero")
    if not 1 <= args.mock_batch_size <= 0xFFFF:
        raise SystemExit("--mock-batch-size must be between 1 and 65535")

    broadcaster = TcpBroadcaster(
        args.host,
        args.port,
        tcp_stats,
        rebatch_samples=args.rebatch,
        rebatch_ms=args.rebatch_ms,
    )
    if args.rebatch > 0:
        logging.info(
            "TCP rebatch enabled: target=%d samples, max_wait=%.0f ms",
            args.rebatch,
            args.rebatch_ms,
        )
    input_client: InputClient

    def on_session_start_mtu(mtu: int) -> None:
        tcp_stats.reset_session(mtu)

    def on_session_start_usb() -> None:
        tcp_stats.reset_session(0)

    if args.transport == "mock":
        tcp_stats.reset_session(0)
        input_client = MockImuClient(
            on_batch=broadcaster.publish_batch,
            sample_rate_hz=args.mock_rate,
            batch_size=args.mock_batch_size,
        )
    elif args.transport == "dual":
        from .dual_client import DualImuClient

        input_client = DualImuClient(
            ble_device_name=args.device_name,
            ble_address=args.address,
            usb_port=args.serial,
            usb_baudrate=args.baudrate,
            on_batch=broadcaster.publish_batch,
            tcp_source=args.tcp_source,
            ble_stats=ble_stats,
            usb_stats=usb_stats,
            scan_timeout_s=args.scan_timeout,
            startup_sync_pings=args.sync_pings,
            refresh_sync_pings=args.sync_refresh_pings,
            refresh_interval_s=args.sync_refresh_interval,
            compare_interval_s=args.stats_interval if args.stats_interval > 0 else 5.0,
        )
    elif args.transport == "usb":
        from .usb_client import UsbImuClient

        input_client = UsbImuClient(
            port=args.serial,
            baudrate=args.baudrate,
            on_batch=broadcaster.publish_batch,
            on_session_start=on_session_start_usb,
            startup_sync_pings=args.sync_pings,
            refresh_sync_pings=args.sync_refresh_pings,
            refresh_interval_s=args.sync_refresh_interval,
        )
    else:
        from .ble_client import XiaoBleClient

        input_client = XiaoBleClient(
            device_name=args.device_name,
            address=args.address,
            on_batch=broadcaster.publish_batch,
            on_session_start=on_session_start_mtu,
            scan_timeout_s=args.scan_timeout,
            startup_sync_pings=args.sync_pings,
            refresh_sync_pings=args.sync_refresh_pings,
            refresh_interval_s=args.sync_refresh_interval,
        )

    await broadcaster.start()

    input_task = asyncio.create_task(input_client.run(), name=f"{args.transport}-client")
    sender_task = asyncio.create_task(broadcaster.run_sender(stop_event), name="tcp-sender")
    stats_task: asyncio.Task[None] | None = None
    if args.stats_interval > 0 and args.transport != "dual":
        stats_task = asyncio.create_task(
            run_stats_reporter(
                tcp_stats,
                args.stats_interval,
                args.idle_warn,
                args.transport,
                stop_event,
            ),
            name="stats-reporter",
        )

    try:
        await input_task
    except asyncio.CancelledError:
        pass
    finally:
        stop_event.set()
        await input_client.stop()
        await broadcaster.flush_rebatch()
        await broadcaster.stop()
        sender_task.cancel()
        if stats_task is not None:
            stats_task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await sender_task
            if stats_task is not None:
                await stats_task
        if args.transport == "dual":
            logging.info("final tcp stats: %s", tcp_stats.format_summary(transport=args.tcp_source))
            logging.info("final ble stats: %s", ble_stats.format_summary(transport="ble"))
            logging.info("final usb stats: %s", usb_stats.format_summary(transport="usb"))
        else:
            logging.info("final stats: %s", tcp_stats.format_summary(transport=args.transport))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Receive XiaoIMU batches over BLE, USB, or a mock source and expose "
            "them on a localhost TCP port."
        ),
    )
    parser.add_argument(
        "--transport",
        choices=("ble", "usb", "dual", "mock"),
        default="ble",
        help="input transport",
    )
    parser.add_argument(
        "--tcp-source",
        choices=("ble", "usb"),
        default="ble",
        help="which transport feeds the TCP stream in dual mode",
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help="TCP bind address")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="TCP bind port")
    parser.add_argument(
        "--serial",
        default=DEFAULT_SERIAL_PORT,
        help="USB serial port, or 'auto' to detect the Xiao (default)",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=115200,
        help="USB serial baud rate",
    )
    parser.add_argument(
        "--mock-rate",
        type=float,
        default=DEFAULT_MOCK_RATE_HZ,
        help="mock samples per second (default: 208)",
    )
    parser.add_argument(
        "--mock-batch-size",
        type=int,
        default=DEFAULT_MOCK_BATCH_SIZE,
        help="samples per generated mock batch (default: 6)",
    )
    parser.add_argument(
        "--device-name",
        default=DEFAULT_DEVICE_NAME,
        help="BLE advertised device name",
    )
    parser.add_argument(
        "--address",
        help="BLE device address (skips name-based scan when set)",
    )
    parser.add_argument(
        "--scan-timeout",
        type=float,
        default=10.0,
        help="seconds to scan for the BLE device",
    )
    parser.add_argument(
        "--stats-interval",
        type=float,
        default=DEFAULT_STATS_INTERVAL_S,
        help="seconds between stats log lines (0 disables)",
    )
    parser.add_argument(
        "--idle-warn",
        type=float,
        default=DEFAULT_IDLE_WARN_S,
        help="warn if no IMU batches arrive for this many seconds",
    )
    parser.add_argument(
        "--sync-pings",
        type=int,
        default=DEFAULT_STARTUP_PINGS,
        help="startup clock-sync ping count before streaming",
    )
    parser.add_argument(
        "--sync-refresh-pings",
        type=int,
        default=DEFAULT_REFRESH_PINGS,
        help="clock-sync ping count for periodic refresh bursts",
    )
    parser.add_argument(
        "--sync-refresh-interval",
        type=float,
        default=DEFAULT_REFRESH_INTERVAL_S,
        help="seconds between periodic clock-sync refresh bursts",
    )
    parser.add_argument(
        "--rebatch",
        type=int,
        default=DEFAULT_REBATCH_SAMPLES,
        help="merge device batches into TCP frames of this many samples (0 disables)",
    )
    parser.add_argument(
        "--rebatch-ms",
        type=float,
        default=DEFAULT_REBATCH_MS,
        help="flush a partial TCP batch after this many milliseconds when rebatching",
    )
    parser.add_argument("--verbose", action="store_true", help="enable debug logging")
    return parser


def configure_logging(verbose: bool, stream: TextIO | None = None) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)s %(message)s",
        stream=stream or sys.stderr,
    )


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    configure_logging(args.verbose)

    try:
        asyncio.run(run_proxy(args))
    except KeyboardInterrupt:
        logging.info("stopped")
        return 0

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
