# hsdaoh-la: hsdaoh la8 / la4 / la48 logic analyzer in ALL LOGIC

Logic analyzer made of

- a Raspberry Pi Pico 2 (RP2350) running a logic analyzer app of
  [hsdaoh-rp2350](https://github.com/dovandung/hsdaoh-rp2350): it samples
  its inputs continuously and streams every sample over HDMI
- an **MS2130** USB3 HDMI capture stick (also MS2131 / MS2130S / MS2131S),
  read on the PC by [libhsdaoh](https://github.com/dovandung/hsdaoh)
  (branch `la`)

| Firmware | Channels | Fastest rate | Shown as |
| --- | --- | --- | --- |
| **la8** (branch `la8`) | GP0..GP7 | 140 MS/s (280 MHz / 2) | la8 (RP2350 + MS2130) |
| **la4** | GP0..GP3 | 280 MS/s (280 MHz / 1) | la4 (RP2350 + MS2130) |
| **la48** (branch `la4`) | 8 or 4, switchable | 140 / 280 MS/s | la48 (RP2350 + MS2130) |

Down to about 4.3 kS/s (divider 65535). la8 sends two 8-bit samples per
16-bit HDMI word, la4 four 4-bit samples; every buffer is unpacked by the
stream format the device reports (PIO_8BIT / PIO_4BIT), so the data is
right whatever firmware runs. This needs libhsdaoh `la` from commit
`25951fc` (Oct 2026) on; with an older libhsdaoh the driver still builds
but reads every stream as la8.

This is a port of the libsigrok driver of the same name
([dovandung/libsigrok](https://github.com/dovandung/libsigrok), branch
`hsdaoh-la`) to ALL LOGIC's libsigrok4DSL. The capture logic (trigger
search, pre-trigger history, lost-data handling) is the same.

## Using it

- The stick shows up in the device list as **la8**, **la4** or **la48
  (RP2350 + MS2130)** when plugged in (the firmware is read over the
  Pico's serial port; without it the device is shown as la8 and the
  number of channels is taken from the stream when it is opened).
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

## la48: 8 or 4 channels

The la48 firmware switches between 8 channels (up to 140 MS/s) and 4
channels (up to 280 MS/s) with `ch 8` / `ch 4` on its USB serial port. In
ALL LOGIC this is the **channel mode** in Device Options:

| Channel mode | Firmware command | Channels | Rates |
| --- | --- | --- | --- |
| Use Channels 0~7 (Max 140MHz) | `ch 8` | D0..D7 | up to 140 MS/s |
| Use Channels 0~3 (Max 280MHz) | `ch 4` | D0..D3 | up to 280 MS/s |

Choosing a mode switches the firmware at once (no re-plug), enables the
mode's channels and updates the sample rate list; going back to 8 channels
at a 4 channel rate raises the divider to the 8 channel minimum. Channels
D4..D7 enabled by hand in the 4 channel mode stay low, and a trigger on
them is ignored (both logged). The maximum rates in the mode names come
from the firmware's status. The libsigrok driver instead picks the mode at
Run from the rate and the enabled channels, as PulseView has no channel
modes.

## Sample rate

The firmware sets the rate: Pico system clock / integer divider.

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
- `../common/capture_core.c/.h` - trigger, pre-trigger history, capture
  buffer, lost-data handling and the LA_CROSS_DATA conversion (shared with
  the cypress-fx3 driver); no ALL LOGIC or libhsdaoh dependency, so it can
  be tested on its own
- `pico_serial.c/.h` - serial port access (libsigrok4DSL has no serial
  layer) and the firmware commands `?`, `div <n>`, `ch 8|4`
