# hsdaoh-la: hsdaoh la8 logic analyzer in ALL LOGIC

8 channel logic analyzer made of

- a Raspberry Pi Pico 2 (RP2350) running the **la8** app of
  [hsdaoh-rp2350](https://github.com/dovandung/hsdaoh-rp2350) (branch `la8`):
  it samples GP0..GP7 continuously and streams every sample over HDMI
- an **MS2130** USB3 HDMI capture stick (also MS2131 / MS2130S / MS2131S),
  read on the PC by [libhsdaoh](https://github.com/dovandung/hsdaoh)
  (branch `la`)

Up to 140 MS/s (system clock 280 MHz / divider 2) down to about 4.3 kS/s
(divider 65535).

This is a port of the libsigrok driver of the same name
([dovandung/libsigrok](https://github.com/dovandung/libsigrok), branch
`hsdaoh-la`) to ALL LOGIC's libsigrok4DSL. The capture logic (trigger
search, pre-trigger history, lost-data handling) is the same.

## Using it

- The stick shows up in the device list as **la8 (RP2350 + MS2130)** when
  plugged in.
- **Windows:** the MS2130 stick needs the WinUSB driver (Zadig: Options >
  List All Devices, select the stick, WinUSB).
- **Trigger:** the simple trigger (0, 1, rising, falling, either edge per
  channel; all conditions true at the same sample). The trigger position
  slider sets how many of the samples are taken before the trigger
  (0..100 %). Without a trigger the capture starts at once.
- The pre- and post-trigger samples are collected in RAM and shown when the
  capture is complete. **Stop** shows what was captured after the trigger
  so far.
- Every HDMI frame carries the Pico's sample counter. If data is lost while
  waiting for the trigger, the pre-trigger history restarts (it never spans
  a gap); if data is lost during a capture, the capture is discarded and
  the driver waits for the trigger again (logged as "Data lost in the HDMI
  stream").

## Sample rate

The la8 firmware sets the rate: Pico system clock / integer divider.

- **With the Pico's USB serial port** the driver reads the firmware status
  when the stick is found and offers the achievable rates (the 6 fastest
  dividers plus the dividers closest to a 1-2-5 series); choosing a rate
  sends `div <n>` to the firmware. The port is found by the Pico SDK's USB
  IDs (2E8A:0009 / 2E8A:000A), else among generic USB serial ports
  ("USB Serial Device (COMx)", /dev/ttyACMx). If that does not work, set
  the environment variable **`HSDAOH_LA_PORT`** (e.g. `COM5`) before
  starting ALL LOGIC. Close any terminal program using the port.
- **Without it** the rate cannot be changed in ALL LOGIC. The driver starts
  the HDMI stream when the device is opened and uses the rate the device
  reports in the stream metadata (ALL LOGIC cannot take a rate change after
  a capture, unlike PulseView).

## Building

The driver is built when CMake finds `libhsdaoh` through pkg-config
(`-DENABLE_HSDAOH_LA=OFF` leaves it out). Build and install libhsdaoh first
(branch `la`; it needs libusb-1.0 and libuvc):

    git clone -b la https://github.com/dovandung/hsdaoh
    cmake -S hsdaoh -B hsdaoh/build -DHSDAOH_BUILD_TOOLS=OFF
    cmake --build hsdaoh/build && cmake --install hsdaoh/build

The CMake summary prints `----- libhsdaoh (hsdaoh-la driver)` when it is
found. On Windows, ship `libhsdaoh.dll` and `libuvc.dll` next to
`AllLogic.exe`.

## Files

- `hsdaoh_la.c` - the driver (device list, settings, acquisition)
- `hsdaoh_la_core.c/.h` - trigger, pre-trigger history, capture buffer,
  lost-data handling and the LA_CROSS_DATA conversion; no ALL LOGIC or
  libhsdaoh dependency, so it can be tested on its own
- `pico_serial.c/.h` - serial port access and the la8 commands
  (libsigrok4DSL has no serial layer)
