# Verification record

Checked on 2026-10-07.

## Completed locally

- Compiled the actual shared firmware math/encoding implementation as native
  C++ using MSVC, and executed it on the host.
- 100,000 exactly-level Madgwick updates remained finite and at identity.
- Zero acceleration/invalid initial quaternion handled without NaN.
- Integrated +90° roll, pitch and yaw with gyro-only updates.
- Accelerometer converged to a known 30° roll.
- Gravity initialisation aligned tilted and exactly upside-down orientations.
- Quaternion reference zero, right-handed axis swaps, reflection/duplicate
  rejection, 416-sample bias subtraction and motion-reset logic passed.
- Golden 20-byte little-endian fixture matched every byte, negative signs,
  full-scale gyro units, saturation and buffer canaries.
- Independently compiled/ran plain Java packet decoding against that C++ fixture.
- Independently ran JavaScript decoding, Euler extraction and vector rotation
  against that same fixture.
- Arduino compiled for `Seeeduino:nrf52:xiaonRF52840Sense` using
  **Seeed nRF52 Boards 1.1.13** with the actual core libraries.
- Android APK assembled and Android lint completed with **0 errors**.
  Remaining warnings concern untranslated English UI strings.
- APK signature verified with Android's `apksigner`; manifest inspected for
  Android 6.0 minimum, API 35 target, legacy Location and Nearby devices
  permissions, and the launcher activity.
- Browser viewer exercised in headless Edge at desktop and phone widths:
  demo animation, angle updates, no horizontal overflow, no JavaScript errors,
  and switching back from demo to offline.
- UF2 converter accepted the application's HEX checksum/address range and
  produced nRF52840-family application blocks.

## Hardware checks still required

No firmware has been flashed onto the user's physical board and no Android
phone was attached during creation. Compilation/host checks do not prove
physical I2C wiring, RGB polarity on a particular revision, LiPo ADC accuracy,
or a particular phone's Bluetooth behavior.

After installing:

1. Reboot with the board stationary; verify blue advertising and ~2-second bias
   calibration. Connect; verify green and approximately 50 samples/s.
2. Verify USB-front / X,Y,Z movement against the model; edit the remap if needed.
3. Check Zero, stationary Calibrate, 20/50/100 Hz, disconnect/reconnect and
   background/resume.
4. With a generic UART client verify CSV LF reconstruction at MTU 23 and
   exactly 20 bytes for binary mode; confirm `t` returns to text.
5. Check denied/granted Bluetooth permissions on the intended phone.
6. Check Battery Level with a connected LiPo; percentage is approximate.
7. A sensor fault should stop samples and blink red while advertising remains
   available. Do not short powered pins to simulate this.
