# Arduino Nano 33 BLE Sense Rev2 — BLE IMU Firmware

Streams 9DoF IMU data over BLE: BMI270 accelerometer/gyroscope samples at
100 Hz and BMM150 magnetometer samples at 20 Hz. The existing 29-byte
accel/gyro characteristic remains compatible with the Controller Driver;
magnetometer data uses a separate 17-byte characteristic. Native Nano BLE
discovery is not yet implemented.

## Prerequisites

```bash
pip install platformio
```

## Build & Flash

Connect the Nano 33 BLE Sense Rev2 to your Mac via USB, then:

```bash
cd motion-controller/firmware/nano_imu_ble
pio run -e nano33ble --target upload
```

If the board isn't detected, **double-tap the white reset button** to enter the
bootloader (the onboard LED will pulse). Then retry the upload command.

## Serial Monitor

Run the wireless orientation viewer from the repository root:

```bash
python3 tools/nano_orientation_viewer/nano_orientation_viewer.py --ble
```

When no BLE central is connected, the firmware also emits comma-separated
9DoF samples over USB serial at 50 Hz.

Each machine-readable line has this layout:

```
9DOF,timestamp_us,gx,gy,gz,ax,ay,az,mx,my,mz
```

Gyroscope values are degrees/second, acceleration values are m/s², and
magnetic-field values are µT. BLE streaming takes priority while connected.

## Host support

Once the firmware is running, the Nano advertises as **"NanoIMU"** over BLE.
The firmware remains supported and maintained here, but the native Controller
Driver currently discovers only `XiaoIMU`. A Nano sensor adapter must use the
Nano service and characteristic UUIDs below while reusing the common 29-byte
packet decoder.

## Firmware Details

| Parameter | Value |
|---|---|
| IMU chip | BMI270 |
| Magnetometer | BMM150 |
| Sample rate | 100 Hz (ODR 100 Hz) |
| Magnetometer sample rate | 20 Hz |
| Gyro range | ±2000 dps |
| Accel range | ±4 g |
| Gyro units (over BLE) | deg/s |
| Accel units (over BLE) | m/s² (converted from g in firmware) |
| BLE device name | `NanoIMU` |
| BLE service UUID | `bfe2b6e1-0005-4583-926c-c39f476f7a34` |
| BLE characteristic UUID | `bfe2b6e1-0006-4583-926c-c39f476f7a34` |
| Magnetometer characteristic UUID | `bfe2b6e1-0007-4583-926c-c39f476f7a34` |
| Packet size | 29 bytes |
| Magnetometer packet size | 17 bytes |

### Packet format (little-endian)

| Offset | Type | Field |
|---|---|---|
| 0 | uint32 | timestamp_us (micros()) |
| 4 | uint8 | sequence (0–255, wrapping) |
| 5 | float32 | gyro X (deg/s) |
| 9 | float32 | gyro Y (deg/s) |
| 13 | float32 | gyro Z (deg/s) |
| 17 | float32 | accel X (m/s²) |
| 21 | float32 | accel Y (m/s²) |
| 25 | float32 | accel Z (m/s²) |

### Magnetometer packet format (little-endian)

| Offset | Type | Field |
|---|---|---|
| 0 | uint32 | timestamp_us (micros()) |
| 4 | uint8 | sequence (0–255, wrapping) |
| 5 | float32 | magnetic field X (µT) |
| 9 | float32 | magnetic field Y (µT) |
| 13 | float32 | magnetic field Z (µT) |

The values use the BMM150's factory trim compensation. Application-level
hard-iron and soft-iron calibration is still required for accurate heading.

### Differences from Xiao firmware

| | Nano 33 BLE Sense Rev2 | Xiao nRF52840 Sense Plus |
|---|---|---|
| IMU | BMI270 + BMM150 | LSM6DS3TR-C |
| BLE stack | ArduinoBLE (Mbed OS) | Adafruit Bluefruit |
| Service UUID | `...0005...` | `...0003...` |
| BLE name | `NanoIMU` | `XiaoIMU` |
| Packet format | Identical 29-byte | Identical 29-byte |
| Magnetometer | Separate 17-byte characteristic | Not available |
