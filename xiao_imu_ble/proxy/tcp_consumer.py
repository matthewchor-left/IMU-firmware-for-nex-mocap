"""Simple TCP client for testing the IMU proxy stream."""

from __future__ import annotations

import argparse
import socket
import sys
import time
from dataclasses import dataclass, field

from .protocol import Frame, FrameStream, TcpImuSample, dropped_samples_in_batch

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765


@dataclass
class StreamStats:
    frames: int = 0
    samples: int = 0
    bytes_received: int = 0
    dropped_samples: int = 0
    last_sequence: int | None = None
    last_sample: TcpImuSample | None = None
    batch_sizes: dict[int, int] = field(default_factory=dict)

    def record_frame(self, frame: Frame) -> None:
        self.frames += 1
        count = len(frame.samples)
        self.samples += count
        self.batch_sizes[count] = self.batch_sizes.get(count, 0) + 1

        self.last_sequence, dropped = dropped_samples_in_batch(
            self.last_sequence,
            frame.samples,
        )
        self.dropped_samples += dropped
        self.last_sample = frame.samples[-1]


def format_sample(sample: TcpImuSample) -> str:
    return (
        f"host_ts={sample.host_timestamp_us} seq={sample.sequence} "
        f"g=({sample.gx:7.2f},{sample.gy:7.2f},{sample.gz:7.2f}) "
        f"a=({sample.ax:6.2f},{sample.ay:6.2f},{sample.az:6.2f})"
    )


def print_stats(stats: StreamStats, elapsed_s: float) -> None:
    if elapsed_s <= 0:
        return

    batch_summary = ", ".join(
        f"{size}x{n}" for size, n in sorted(stats.batch_sizes.items())
    )
    last = stats.last_sample
    last_line = format_sample(last) if last is not None else "no samples yet"

    print(
        f"[{elapsed_s:5.1f}s] v2 "
        f"{stats.frames / elapsed_s:6.1f} frames/s, "
        f"{stats.samples / elapsed_s:7.1f} samples/s, "
        f"{stats.bytes_received / elapsed_s:8.0f} B/s, "
        f"dropped={stats.dropped_samples}, "
        f"batches={{{batch_summary}}}"
    )
    print(f"  latest: {last_line}")


def run_consumer(args: argparse.Namespace) -> int:
    stream = FrameStream()
    stats = StreamStats()
    started = time.monotonic()
    last_report = started

    print(
        f"connecting to {args.host}:{args.port} "
        f"(stats every {args.stats_interval:.1f}s)",
        file=sys.stderr,
    )

    with socket.create_connection((args.host, args.port), timeout=args.connect_timeout) as sock:
        sock.settimeout(args.read_timeout)
        print("connected", file=sys.stderr)

        while True:
            if args.max_seconds is not None and time.monotonic() - started >= args.max_seconds:
                break

            try:
                chunk = sock.recv(args.recv_size)
            except socket.timeout:
                if args.print_samples:
                    continue
                print("read timeout (no data received)", file=sys.stderr)
                return 1

            if not chunk:
                print("server closed connection", file=sys.stderr)
                break

            stats.bytes_received += len(chunk)

            for frame in stream.feed(chunk):
                stats.record_frame(frame)
                if args.print_samples:
                    for sample in frame.samples:
                        print(format_sample(sample))

            now = time.monotonic()
            if now - last_report >= args.stats_interval:
                print_stats(stats, now - started)
                last_report = now

    print_stats(stats, time.monotonic() - started)
    if stats.samples == 0:
        print("no IMU samples received", file=sys.stderr)
        return 1

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Connect to the IMU TCP proxy and print stream statistics.",
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help="proxy TCP host")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="proxy TCP port")
    parser.add_argument(
        "--stats-interval",
        type=float,
        default=1.0,
        help="seconds between summary lines",
    )
    parser.add_argument(
        "--print-samples",
        action="store_true",
        help="print every decoded sample (very verbose at ~208 Hz)",
    )
    parser.add_argument(
        "--recv-size",
        type=int,
        default=4096,
        help="socket recv buffer size",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=5.0,
        help="TCP connect timeout in seconds",
    )
    parser.add_argument(
        "--read-timeout",
        type=float,
        default=5.0,
        help="socket read timeout in seconds",
    )
    parser.add_argument(
        "--max-seconds",
        type=float,
        help="stop after this many seconds (for quick smoke tests)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        return run_consumer(args)
    except KeyboardInterrupt:
        print("\nstopped", file=sys.stderr)
        return 0
    except OSError as exc:
        print(f"connection error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
