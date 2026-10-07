# XIAO Motion

Motion sensing for the **Seeed Studio XIAO nRF52840 Sense** with the LSM6DS3TR-C.
Includes firmware, an installable Android app, and a Web Bluetooth viewer.

- [Download the Android APK](https://github.com/ShahabSabzi/xiao-motion-sense-2026-10-07/releases/latest/download/xiao-motion-debug.apk)
- [Download drag-and-drop firmware](https://github.com/ShahabSabzi/xiao-motion-sense-2026-10-07/releases/latest/download/XiaoMotion.uf2)
- [Open the browser viewer](https://shahabsabzi.github.io/xiao-motion-sense-2026-10-07/)
- [All release files](https://github.com/ShahabSabzi/xiao-motion-sense-2026-10-07/releases/latest)

## Install and start moving

1. **Flash the board:** connect the Sense by USB, double-tap RESET to enter its
   UF2 bootloader, and copy `XiaoMotion.uf2` onto the new bootloader drive.
   The drive disappearing after the copy is normal. This firmware targets the
   original nRF52840 Sense, not the Sense Plus or a plain non-Sense XIAO.
   The UF2 contains the application only and uses the board's existing
   bootloader and S140 SoftDevice. If your bootloader does not expose a UF2
   drive, use the Arduino upload method below.
2. **Install the phone app:** download `xiao-motion-debug.apk` on an Android
   6.0+ phone. Open it and allow installation from that browser/file manager
   if Android asks. This is a debug-signed development app.
3. Keep the board motionless for about **two seconds** at boot. Movement
   restarts the calibration window. Blue blinking means advertising.
4. Open **XIAO Motion**, tap **Connect**, allow **Nearby devices** on
   Android 12+, or **Location** on Android 6–11, then select `XIAO-IMU`.
   Android 6–11 also needs the system Location switch enabled for scanning.
   The app never obtains your position and has no Internet permission.
5. Move the board. Green means connected. The board model, five-second
   acceleration/angular-rate charts, and roll/pitch/yaw update live.

**Zero** sets the current quaternion as the orientation reference.
**Calibrate** repeats the stationary gyro measurement and resets the
orientation reference. The rate selector sends 20, 50 or 100 Hz commands.
The app disconnects when it goes into the background; tap Connect on return.

The browser viewer offers the same controls and rendering. Open it over
HTTPS in a Web Bluetooth-capable Chrome/Edge browser (including supported
Android Chrome). **Preview demo** is explicitly simulated motion. Browser
support varies; Safari/Firefox do not provide this viewer's Bluetooth API.
Only one viewer can connect to this one-peripheral firmware at a time.

## Firmware / Arduino upload

Install **Seeed nRF52 Boards 1.1.13**, choose **Seeed XIAO nRF52840 Sense**,
and open `firmware/XiaoMotion/XiaoMotion.ino`. Keep `MotionMath.h` beside it.
Use the **non-mbed** core: it supplies Bluefruit, Wire and TinyUSB.
There are no additional sensor/fusion libraries to install.

Board Manager URL:

```text
https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
```

For Arduino CLI:

```sh
arduino-cli core update-index --additional-urls https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
arduino-cli core install Seeeduino:nrf52@1.1.13 --additional-urls https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense firmware/XiaoMotion
arduino-cli upload --fqbn Seeeduino:nrf52:xiaonRF52840Sense --port YOUR_PORT firmware/XiaoMotion
```

The sketch starts Bluefruit advertising **before** it powers/checks the IMU.
It drives `PIN_LSM6DS3TR_C_POWER` high and waits 50 ms, checks SDA 17 and
SCL 16 as plain inputs, and initialises **Wire1** only when both are high.
It never initialises or probes the external D4/D5 bus.

The IMU register driver is in the sketch. It configures BDU, auto increment,
208 Hz for both sensors, ±8 g and ±2000 degrees/second, and reads a coherent
12-byte gyro/accelerometer burst when both data-ready bits are set. It uses
Wire1's TWIM1 peripheral with a 3 ms transfer deadline and bounded error
cleanup because this core's ordinary I2C calls can wait forever. Sensor
failure stops samples and blinks red, while Bluetooth remains available.
Power-cycle after resolving a sensor/bus fault.

Fusion runs at the sensor sampling rate using measured elapsed time, separate
from the requested BLE stream rate. Gyro bias uses 416 consecutive stationary
samples; acceleration must be near 1 g and all gyro axes below 5 degrees/s.
The Madgwick IMU update guards both zero acceleration and a zero gradient.
Initial roll/pitch comes from gravity, including upside-down boots; yaw starts
at zero. The transmitter returns immediately when BLE buffers are busy,
dropping stale binary samples and finishing partial CSV lines in order.

### Mounting and axes

At the top of the sketch, edit:

```cpp
const int8_t AXIS_REMAP[3] = {1,2,3};
const Quat MOUNT_ROTATION = {1,0,0,0};
```

The three signed entries select native sensor input X/Y/Z (1/2/3) for output
X/Y/Z. Examples:

| Table | Mapping |
|---|---|
| `{1,2,3}` | Identity |
| `{2,-1,3}` | X ← Y, Y ← −X, Z ← Z |
| `{1,-2,-3}` | Flip Y and Z |
| `{-1,2,-3}` | Flip X and Z |

Keep a right-handed frame: a single sign flip alone is a reflection rather
than a physical rotation. Invalid/duplicate mappings are rejected as a
configuration fault. For a non-90-degree mounting angle, supply the additional
unit quaternion `MOUNT_ROTATION` in **w,x,y,z** order. It rotates the remapped
sensor vectors into your desired board axes; e.g. a +30° rotation about X is
`{0.9659258f,0.2588190f,0,0}`. Bias is measured/subtracted in the native
frame before remapping. Both clients use identical quaternion rotation and
camera conventions. Their schematic board model has its USB socket toward −Y,
with +Z out of its top. Adjust the table for your chosen physical mounting.

### LEDs and battery

| LED | Meaning |
|---|---|
| Blue blinking | Advertising |
| Green solid | Connected (also during calibration) |
| Red blinking | IMU, bus or axis-map fault; Bluetooth still works |

RGB outputs are active-low; the core's misleading `LED_STATE_ON` macro is
deliberately not used. The standard Battery Service reports an approximate
single-cell LiPo percentage every 10 seconds. The sketch enables the onboard
voltage divider via `VBAT_ENABLE`, reads `PIN_VBAT` with a 3.0 V ADC range,
and uses the 1 MΩ / 510 kΩ divider ratio. Percentage is meaningful with a
battery attached and varies under load/charging; this is not a fuel gauge.

## Protocol

Nordic UART Service:
`6e400001-b5a3-f393-e0a9-e50e24dcca9e`

- RX (write commands): `6e400002-b5a3-f393-e0a9-e50e24dcca9e`
- TX (subscribe to notifications): `6e400003-b5a3-f393-e0a9-e50e24dcca9e`
- Battery Service: `180F`, Battery Level: `2A19`

On boot and each fresh connection, the stream defaults to text at 20 Hz.
One LF-terminated CSV line, no headers/status lines on UART:

```text
ax,ay,az,gx,gy,gz,roll,pitch,yaw
```

Acceleration is in **g, including gravity**, angular rates in **degrees/s**,
Euler angles in **degrees**. At the smallest MTU, a CSV line spans several
notifications: concatenate bytes until LF.

Binary: exactly **20 bytes per notification**, ten signed 16-bit integers
in little-endian order. No header, delimiter, timestamp or checksum.

| Byte offset | Values | Decode |
|---|---|---|
| 0, 2, 4, 6 | qw, qx, qy, qz | divide by 16384 |
| 8, 10, 12 | ax, ay, az | divide by 1000 → g |
| 14, 16, 18 | gx, gy, gz | divide by 16 → degrees/s |

Quaternion maps body vectors into world coordinates. Rotation uses a
right-handed frame with gravity along world +Z; Euler extraction uses
`Rz(yaw) Ry(pitch) Rx(roll)`. After `z`, the reported rotation is
`conjugate(q_at_zero) * q_now`; acceleration/rate axes are unchanged.
Values round and saturate rather than wrapping on conversion.

Write ASCII commands to RX, without needing a newline:

| Command | Effect |
|---|---|
| `b` | Binary mode, resets stream rate to 50 Hz |
| `t` | Text mode, resets stream rate to 20 Hz |
| `z` | Zero all three orientation axes using a quaternion reference |
| `c` | Recalibrate gyro bias; keep still; stream pauses until completion |
| `1`, `2`, `3` | Set stream rate to 20, 50, 100 Hz in either mode |

Unknown bytes (including CR/LF) are ignored. No command acknowledgement is
inserted into the sample stream. Subscribing and mode commands work even
during initial calibration; valid samples start when calibration finishes.

The Android app serialises GATT discovery, notification subscription, commands
and battery reads, has operation timeouts, ignores obsolete callbacks from
old connections, and sends `b` only after subscription acknowledgement.
Neither client requests an MTU change: 20 bytes fits ATT MTU 23.
A phone can choose a slower connection interval; observed sample rate may
fall below the selected rate on a congested BLE link.

## GitHub builds and Pages

`.github/workflows/build.yml` compiles/runs the host math tests, independently
decodes the generated fixture in Java and JavaScript, compiles the firmware,
builds/lints the Android debug APK, verifies its signature, and retains the
installables as an Actions artifact.

Push a version tag such as `v1.0.1` to attach the APK, UF2, HEX, DFU ZIP and
SHA-256 checksums to a GitHub release. Alternatively, run **Build, verify, and
release APK** from Actions with a `release_tag`. Empty input builds an artifact
only. The job uses the runner's existing SDK and `sdkmanager`; it does **not**
use `android-actions/setup-android`. The Linux firmware job installs Adafruit�s DFU packaging tool in an isolated
Python 3.11 environment. Java 17, Gradle 8.11.1, AGP 8.9.2 and
Android SDK/build tools 35 are pinned. No Kotlin, AndroidX, Compose or
third-party runtime libraries are included.

The debug signing key in `android/debug.keystore` is **intentionally public**
and only for this development app. It keeps local and CI builds update-compatible.
Future production distribution requires a private release key and a separate
release configuration.

`.github/workflows/pages.yml` publishes `web/` on main changes. If you fork
this repository, enable **Settings → Pages → Source: GitHub Actions**, update
the links in this README and `web/index.html`, and enable the Pages job for
your fork by removing its fork condition.

## Verification and practical limits

See [VALIDATION.md](VALIDATION.md) for what was actually checked. To run the
portable host tests on Linux/macOS with a C++ compiler, Java and Node:

```sh
mkdir -p build/host
g++ -std=c++11 -Wall -Wextra -pedantic tests/motion_test.cpp -o build/host/motion-test
(cd build/host && ./motion-test)
javac -d build/host android/app/src/main/java/org/xiao/motion/MotionPacket.java tests/PacketTest.java
java -cp build/host org.xiao.motion.PacketTest build/host/packet.bin
node tests/protocol_test.mjs build/host/packet.bin
```

An accelerometer/gyro IMU measures orientation and motion signals, **not
absolute 3D position**. The 3D view rotates the board; it does not estimate
translation. Yaw is relative and drifts without a magnetometer. Sustained
linear acceleration also affects accelerometer-based tilt correction. Zero
provides a convenient relative reference, not a heading measurement.
