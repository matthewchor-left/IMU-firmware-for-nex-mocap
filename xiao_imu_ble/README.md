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

BLE firmware (default):

```bash
pio run -e xiaoblesense_adafruit --target upload
```

USB CDC streaming firmware:

```bash
pio run -e xiaoblesense_usb --target upload
```

Dual BLE + USB firmware (for transport verification):

```bash
pio run -e xiaoblesense_dual --target upload
```

The USB build streams binary `XIMU` frames over the USB serial port in batches of
16 samples. It does not print text on the CDC port. Dual mode streams the same
samples on both transports with a shared sequence counter.

If the board isn't detected, **double-tap the tiny reset button** to enter the
UF2 bootloader. It will appear as a USB mass-storage device. Then retry the
upload command.

## Serial Monitor

Optional for the BLE build only:

```bash
pio device monitor
```

Do not use the serial monitor while the USB proxy is connected to the device.

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
| Packet size | 29 bytes per sample, adaptive batching up to 6 samples / 174 bytes per BLE notification |

### Packet format (little-endian)

| Offset | Type | Field |
|--------|------|-------|
| 0 | uint32 | timestamp_us (micros()) |
| 4 | uint8 | sequence (0–255, wrapping) |
| 5 | float32 | gyro X (deg/s) |
| 9 | float32 | gyro Y (deg/s) |
| 13 | float32 | gyro Z (deg/s) |
| 17 | float32 | accel X (m/s²) |
| 21 | float32 | accel Y (m/s²) |
| 25 | float32 | accel Z (m/s²) |

`timestamp_us` is a wrapping `micros()` value (about 71.6 minutes per wrap).
The host unwraps it before applying the calibrated clock mapping.

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

A Python proxy bridges the BLE stream to a localhost TCP port so other programs
can consume IMU batches without implementing BLE themselves.

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

USB (after flashing `xiaoblesense_usb`):

```bash
python -m proxy --transport usb --serial /dev/ttyACM0
```

Dual verification (after flashing `xiaoblesense_dual`):

```bash
python -m proxy --transport dual --serial /dev/ttyACM0 --tcp-source ble
```

Dual mode runs independent BLE and USB clock sync, compares offset/scale and
matches samples by `sequence`, and forwards one transport to TCP (`--tcp-source
ble|usb`, default `ble`).

USB uses the same clock-sync algorithm as BLE (startup ping burst + periodic
refresh) and emits TCP protocol v2 with host-aligned timestamps by default.
Use `--raw-timestamps` for device `micros()` without sync (not supported in dual
mode).

Optional flags:

- `--transport ble|usb` — input source (default `ble`)
- `--serial /dev/ttyACM0` — USB serial port
- `--device-name XiaoIMU` — BLE advertised name (default)
- `--address AA:BB:CC:DD:EE:FF` — connect by address instead of scanning by name
- `--stats-interval 5` — log throughput and drop counters every N seconds (default `5`, `0` disables)
- `--idle-warn 3` — warn if no IMU batches arrive for N seconds after connect
- `--raw-timestamps` — skip clock sync and emit protocol v1 with device `micros()`
- `--sync-pings 50` — startup clock-sync ping count
- `--verbose` — debug logging

On Linux the proxy negotiates BLE MTU after connect so the device can batch up to
6 samples per notification. If MTU stays below 32, the firmware will not stream.

Only one BLE central can connect to the Xiao at a time. Do not run the proxy
while the Motion Controller is already connected to the device.

### USB CDC frame format

Each serial message from the USB firmware uses an 8-byte header:

| Offset | Type | Field |
|--------|------|-------|
| 0 | 4 bytes | magic `XIMU` |
| 4 | uint8 | version (`1`) |
| 5 | uint8 | `msg_type` |
| 6 | uint16 LE | `payload_len` |
| 8 | `payload_len` | payload |

Message types:

| `msg_type` | Direction | Payload |
|------------|-----------|---------|
| `0x01` | device → host | `N × 29` IMU samples |
| `0x02` | host → device | `u32 request_id` (clock-sync ping) |
| `0x03` | device → host | `u32 request_id`, `u32 timestamp_us` (clock-sync reply) |

The proxy converts USB batches into the same TCP framing used for BLE.

### TCP frame format

Each TCP message is one BLE notification batch, wrapped with a small header:

| Offset | Type | Field |
|--------|------|-------|
| 0 | 4 bytes | magic `XIMU` |
| 4 | uint8 | version (`1` or `2`) |
| 5 | uint16 LE | `sample_count` |
| 7 | `sample_count × sample_size` | samples |

**Version 2 (default)** — host-aligned timestamps after BLE clock sync:

```
[u64 host_timestamp_us][u8 sequence][f32 gx][f32 gy][f32 gz][f32 ax][f32 ay][f32 az]
```

`host_timestamp_us` is mapped to the proxy host's monotonic clock (microseconds).
The proxy runs a startup sync burst (50 pings by default) and periodic refresh
bursts before forwarding IMU data.

**Version 1** — raw device timestamps (`--raw-timestamps`):

```
[u32 device_timestamp_us][u8 sequence][f32 gx][f32 gy][f32 gz][f32 ax][f32 ay][f32 az]
```

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
