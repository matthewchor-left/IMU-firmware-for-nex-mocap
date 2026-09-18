# Xiao nRF52840 Sense Plus — BLE IMU Firmware

Streams 6-axis IMU data (LSM6DS3TR-C) over BLE at ~208 Hz.
The packet format is consumed by the native Motion Controller's XIAO sensor
adapter.

## Prerequisites

```bash
pip install platformio
```

## Build & Flash

Connect the Xiao via USB-C, then:

```bash
pio run --target upload
```

The firmware streams the same IMU samples over **both** BLE and USB with a shared
sequence counter. USB uses binary `XIMU` frames in batches of 16 samples; the
CDC port does not print text.

If the board isn't detected, **double-tap the tiny reset button** to enter the
UF2 bootloader. It will appear as a USB mass-storage device. Then retry the
upload command.

Do not use `pio device monitor` while the USB proxy is connected — the CDC port
carries binary frames only.

## Usage with Motion Controller

Once the firmware is running, the Xiao advertises as **"XiaoIMU"** over BLE.
Build and launch the native dashboard from `motion-controller/`:

```bash
cmake --preset macos-arm64-dashboard
cmake --build --preset macos-arm64-dashboard
build/macos-arm64-dashboard/apps/dashboard/MotionController.app/Contents/MacOS/MotionController \
  --live --camera-index 0
```

The Controller Driver discovers the Xiao by its device and service UUIDs,
normalizes the samples, and maps the wrapping device clock to the host clock.

## Firmware Details

| Parameter | Value |
|-----------|-------|
| IMU chip | LSM6DS3TR-C |
| Sample rate | ~208 Hz |
| Gyro range | ±1000 dps |
| Accel range | ±8 g |
| Gyro units (over BLE) | deg/s |
| Accel units (over BLE) | m/s² |
| BLE device name | `XiaoIMU` |
| BLE service UUID | `bfe2b6e1-0003-4583-926c-c39f476f7a34` |
| IMU characteristic UUID | `bfe2b6e1-0004-4583-926c-c39f476f7a34` |
| Sync characteristic UUID | `bfe2b6e1-0005-4583-926c-c39f476f7a34` |
| Packet size | 29 bytes per sample (see [Protocol reference](#protocol-reference)) |
| BLE batching | up to 6 samples / notification (MTU-dependent) |
| USB batching | up to 16 samples / CDC frame |

### Clock synchronization protocol

The sync characteristic accepts a 4-byte little-endian `uint32` request ID.
Its write callback captures `micros()` immediately and notifies an 8-byte
response:

| Offset | Type | Field |
|--------|------|-------|
| 0 | uint32 | request ID |
| 4 | uint32 | timestamp_us at request receipt |

Before camera capture starts, the Controller Driver sends 50 pings and estimates the
host/device clock offset from the five responses with the lowest round-trip
time. Short background bursts periodically refresh an affine clock model to
account for oscillator drift. Request IDs prevent delayed BLE responses from
being matched to the wrong ping.

The result aligns software timestamps; it does not identify the camera
sensor's physical exposure instant and is not a replacement for hardware
triggering when sub-millisecond synchronization is required.

## TCP Proxy

A Python proxy bridges BLE or USB device streams to a localhost TCP port so
other programs can consume IMU batches without implementing those transports
themselves. Both inputs are normalized to the same TCP framing (see
[Protocol reference](#protocol-reference)).

### Setup

```bash
pip install -r requirements-proxy.txt
```

On Linux you may need permission to use Bluetooth (for example, membership in
the `bluetooth` group).

### Run

BLE (default):

```bash
python -m proxy --host 127.0.0.1 --port 8765
```

USB:

```bash
python -m proxy --transport usb
```

Mock data (no IMU device required):

```bash
python -m proxy --transport mock
```

Mock mode generates deterministic, moving 6-axis data at 208 samples/s in
batches of 6 by default. It uses the same host-timestamped TCP v2 format as the
hardware transports, so existing consumers can connect without changes:

```bash
python -m proxy --transport mock --mock-rate 100 --mock-batch-size 10
```

Dual verification (BLE + USB together):

```bash
python -m proxy --transport dual --tcp-source ble
```

Dual mode runs independent BLE and USB clock sync, compares offset/scale and
matches samples by `sequence`, and forwards one transport to TCP (`--tcp-source
ble|usb`, default `ble`).

USB uses the same clock-sync algorithm as BLE (startup ping burst + periodic
refresh). The TCP stream always carries host-aligned timestamps (protocol v2).

Optional flags:

- `--transport ble|usb|dual|mock` — input source (default `ble`)
- `--tcp-source ble|usb` — which transport feeds TCP in dual mode (default `ble`)
- `--serial auto` — USB serial port (`auto` detects the Xiao; or e.g. `/dev/ttyACM1`)
- `--mock-rate 208` — samples per second in mock mode
- `--mock-batch-size 6` — generated samples per batch in mock mode
- `--device-name XiaoIMU` — BLE advertised name (default)
- `--address AA:BB:CC:DD:EE:FF` — connect by address instead of scanning by name
- `--stats-interval 5` — log throughput and drop counters every N seconds (default `5`, `0` disables)
- `--idle-warn 3` — warn if no IMU batches arrive for N seconds after connect
- `--sync-pings 50` — startup clock-sync ping count
- `--rebatch 32` — merge samples into larger TCP frames (0 = disabled)
- `--rebatch-ms 50` — flush partial rebatch after N ms when rebatching
- `--verbose` — debug logging

On Linux the proxy negotiates BLE MTU after connect so the device can batch up to
6 samples per notification. If MTU stays below 32, the firmware will not stream.

Only one BLE central can connect to the Xiao at a time. Do not run the proxy
while the Motion Controller is already connected to the device.

### Protocol reference

All transports share the same 29-byte IMU sample record. The Python proxy
normalizes BLE and USB device batches into **one TCP wire format** on
`127.0.0.1:8765` (configurable). TCP consumers do not need to know whether the
proxy input was BLE or USB.

```
Device (BLE notify or USB CDC)     Proxy                         TCP client
─────────────────────────────      ─────                         ──────────
N × 29-byte samples/batch    →     clock sync              →     XIMU frame
                                   host-aligned v2               (33 B/sample)
```

#### IMU sample record (29 bytes, little-endian)

Used inside BLE notifications and USB `0x01` batches.

| Offset | Size | Type | Field |
|--------|------|------|-------|
| 0 | 4 | uint32 | `timestamp_us` — device `micros()` at sample time |
| 4 | 1 | uint8 | `sequence` — 0–255, wrapping sample counter |
| 5 | 4 | float32 | `gx` — gyro X (deg/s) |
| 9 | 4 | float32 | `gy` — gyro Y (deg/s) |
| 13 | 4 | float32 | `gz` — gyro Z (deg/s) |
| 17 | 4 | float32 | `ax` — accel X (m/s²) |
| 21 | 4 | float32 | `ay` — accel Y (m/s²) |
| 25 | 4 | float32 | `az` — accel Z (m/s²) |

`timestamp_us` wraps every ~71.6 minutes. The proxy unwraps it before clock
mapping. Gyro/accel units are identical on BLE and USB.

#### BLE input (device → proxy)

| Item | Value |
|------|-------|
| Delivery | GATT notification on IMU characteristic |
| UUID | `bfe2b6e1-0004-4583-926c-c39f476f7a34` |
| Payload | `N × 29` bytes, no extra header |
| Batch size | 1–6 samples (MTU-dependent; up to 6 at MTU ≥ 177) |
| Rate | ~208 Hz aggregate |

Clock sync uses the sync characteristic (`bfe2b6e1-0005-4583-926c-c39f476f7a34`):

| Direction | Payload |
|-----------|---------|
| host → device (write) | `u32 request_id` (4 bytes) |
| device → host (notify) | `u32 request_id`, `u32 timestamp_us` (8 bytes) |

#### USB input (device → proxy)

USB CDC messages are framed. Each message has an 8-byte header followed by a
payload:

| Offset | Size | Type | Field |
|--------|------|------|-------|
| 0 | 4 | char[4] | magic `XIMU` |
| 4 | 1 | uint8 | `version` (`1`) |
| 5 | 1 | uint8 | `msg_type` |
| 6 | 2 | uint16 LE | `payload_len` |
| 8 | `payload_len` | bytes | payload |

Message types:

| `msg_type` | Direction | Payload |
|------------|-----------|---------|
| `0x01` IMU batch | device → host | `N × 29` IMU samples |
| `0x02` SYNC_REQ | host → device | `u32 request_id` (4 bytes) |
| `0x03` SYNC_RESP | device → host | `u32 request_id`, `u32 timestamp_us` (8 bytes) |

IMU batches carry up to **16 samples** (`USB_BATCH_CAPACITY`). The proxy only
forwards `0x01` messages to TCP; sync messages stay on the serial link.

#### TCP output (proxy → consumer)

**BLE and USB produce the same TCP format.** By default the proxy forwards one
device batch per TCP frame. Use `--rebatch` to merge smaller device batches into
larger TCP frames (useful for non-real-time consumers).

| Proxy input | Default `sample_count` per TCP frame |
|-------------|--------------------------------------|
| `--transport ble` | 1–6 (device batch size) |
| `--transport usb` | up to 16 |
| `--transport dual --tcp-source ble` | 1–6 |
| `--transport dual --tcp-source usb` | up to 16 |
| any transport + `--rebatch 32` | up to 32 (see below) |

**Rebatch policy** (`--rebatch N`, optional `--rebatch-ms`):

- Accumulate aligned samples until `N` are ready, then emit one TCP frame.
- If fewer than `N` samples are buffered, flush the partial batch after
  `--rebatch-ms` milliseconds (default `50`).
- On proxy shutdown, any remaining samples are flushed.

Example for offline processing:

```bash
python -m proxy --transport ble --rebatch 32 --rebatch-ms 50
```

TCP frame layout:

| Offset | Size | Type | Field |
|--------|------|------|-------|
| 0 | 4 | char[4] | magic `XIMU` |
| 4 | 1 | uint8 | `version` (`2`) |
| 5 | 2 | uint16 LE | `sample_count` |
| 7 | `sample_count × 33` | bytes | concatenated samples |

Parse TCP as a byte stream: scan for `XIMU`, read the 7-byte header, then read
`sample_count × 33` bytes. `FrameStream` in `proxy/protocol.py` implements this.

Each TCP sample is **33 bytes**:

| Offset | Size | Type | Field |
|--------|------|------|-------|
| 0 | 8 | uint64 | `host_timestamp_us` — proxy host monotonic clock (µs) |
| 8 | 1 | uint8 | `sequence` |
| 9 | 4 | float32 | `gx` |
| 13 | 4 | float32 | `gy` |
| 17 | 4 | float32 | `gz` |
| 21 | 4 | float32 | `ax` |
| 25 | 4 | float32 | `ay` |
| 29 | 4 | float32 | `az` |

`host_timestamp_us` is derived from the device `timestamp_us` via an affine map
estimated by startup + periodic sync pings (50 + 10 pings by default). BLE and
USB each run their own mapper; TCP carries the mapper for the selected transport.

Maximum frame size: `7 + 16 × 33 = 535` bytes (USB input).

##### TCP worked example (2 samples)

```
58 49 4d 55   magic "XIMU"
02            version 2
02 00         sample_count = 2
[33 bytes]    sample 0
[33 bytes]    sample 1
```

Total frame length: `7 + 2 × 33 = 73` bytes.

### Test consumer

With the proxy running, start the bundled TCP consumer in another terminal:

```bash
python -m proxy.tcp_consumer
```

It prints one summary line per second (frames/s, samples/s, batch sizes, latest
sample). Useful flags:

- `--print-samples` — print every decoded sample
- `--max-seconds 5` — exit after a short smoke test
- `--host` / `--port` — override the proxy address
