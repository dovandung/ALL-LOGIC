# Development report: dovandung/ALL-LOGIC

This fork of [sipeed/ALL-LOGIC](https://github.com/sipeed/ALL-LOGIC) adds
two new logic analyzer drivers, ported from libsigrok, and speeds up two
hot functions of the Sipeed SLogic driver. The work was done with Claude
(Anthropic) in October 2026, starting from Sipeed commit `c914f7c`.

| Commit | Change |
| --- | --- |
| `186ad6a` | SLogic: vectorize `slogic_unpack_range` below 8 channels |
| `a692080` | SLogic: speed up `slogic_trigger_scan` |
| `e77be66` | New driver **hsdaoh-la** (RP2350 la8 + MS2130 HDMI capture stick) |
| `219cfe4` | New driver **cypress-fx3** (Cypress FX3, 8 channels) with FX3 firmware upload |
| `2a8939f` | cypress-fx3 uses libsigrok's `ezusb_upload_firmware_fx3()` |
| `1c04ab0` | cypress-fx3: choice of the 8 or 16 channel firmware |

In total: 20 files changed, about 4,200 lines added.

**Status on real hardware:** the hsdaoh-la driver and the cypress-fx3
driver (both firmwares) have been confirmed working by the owner on the
real devices. The SLogic speed-ups were measured with benchmarks on the
build machine only.

---

## 1. SLogic speed-ups

Both functions are in
`libsigrok4DSL/hardware/sipeed-slogic/slogic/slogic_data.c`. The results
are byte-for-byte identical to the original code.

### `slogic_unpack_range` (commit `186ad6a`)

Expands packed 1, 2 and 4 channel samples to one byte per sample, for every
delivered sample.

- **x86 / x64:** SSE2, 16 packed bytes per step.
- **Other little-endian CPUs:** 64-bit bit tricks, 8 samples per store.
- Partial bytes at either end keep the original code; `SLOGIC_NO_SIMD`
  forces the portable path. 8, 16 and 32 channels are unchanged (they were
  already a `memcpy`).

| Channels | Before | After | Speed-up |
| --- | --- | --- | --- |
| 1 | 1.3 GSa/s | 12.6 GSa/s | ×9.6 |
| 2 | 1.2 GSa/s | 17.2 GSa/s | ×14.8 |
| 4 | 2.7 GSa/s | 12.1 GSa/s | ×4.5 |

**Checked:** 120,000 random cases for 1/2/4/8/16/32 channels, with and
without SIMD, identical to the original; AddressSanitizer confirms it
still never reads past the last sample.

### `slogic_trigger_scan` (commit `a692080`)

Finds the trigger sample in the stream.

- A 64-bit word that changes on the trigger channels is now tested for all
  its samples at once (before: one sample at a time, up to 64 per word).
- Quiet stretches are skipped 64 bytes at a time (SSE2) or 32 bytes
  (portable); this is only attempted right after a quiet word, so busy
  signals do not pay for it.

| Channels | Quiet signal | Busy trigger channel |
| --- | --- | --- |
| 1 | 4.9 → 10.1 GB/s | 4.4 → 9.6 GB/s |
| 2 | 4.6 → 8.5 GB/s | 0.31 → 2.2 GB/s (×7) |
| 4 | 5.0 → 8.9 GB/s | 0.57 → 2.3 GB/s (×4) |
| 8 | 5.1 → 9.9 GB/s | 0.90 → 2.3 GB/s |
| 16 | 3.2 → 5.8 GB/s | 1.3 → 2.3 GB/s |
| 32 | 5.1 → 9.3 GB/s | 2.1 → 2.3 GB/s |

**Checked:** 106,000 random scans (all channel counts, level and edge
triggers, signal densities, start positions, resumed scans), identical to
the original including the "previous sample" carried to the next call.

---

## 2. hsdaoh-la driver (commit `e77be66`)

8 channel logic analyzer: a Raspberry Pi Pico 2 (RP2350) running the
**la8** firmware of
[hsdaoh-rp2350](https://github.com/dovandung/hsdaoh-rp2350) streams
GP0..GP7 over HDMI into an **MS2130** USB3 capture stick, read with
[libhsdaoh](https://github.com/dovandung/hsdaoh) (branch `la`). Up to
140 MS/s.

Ported from the libsigrok driver
([dovandung/libsigrok](https://github.com/dovandung/libsigrok), branch
`hsdaoh-la`). The capture logic is unchanged: trigger on the PC,
pre- and post-trigger samples in RAM, lost HDMI data restarts the
pre-trigger history and discards a capture in progress.

What had to change for ALL LOGIC:

- **Detection** in ALL LOGIC's own USB list, so plugging and unplugging
  works; the stick IDs `345F:2130..2133` were added to the hotplug list.
- **Trigger** from ALL LOGIC's simple trigger panel and position slider.
- **Data** in ALL LOGIC's `LA_CROSS_DATA` format.
- **Pico serial port:** ALL LOGIC has no serial layer and no way to pick a
  port, so `pico_serial.c` finds the Pico by its USB IDs (or the
  `HSDAOH_LA_PORT` environment variable) and sends the la8 `?` and
  `div <n>` commands, on Windows, Linux and macOS.
- **Sample rate without the serial port:** ALL LOGIC ignores rate updates
  after a capture, so the stream is started when the device is opened and
  the rate is read from the HDMI metadata first.
- **Optional build:** compiled only when CMake finds libhsdaoh
  (`-DENABLE_HSDAOH_LA=OFF` leaves it out).

Files: `libsigrok4DSL/hardware/hsdaoh-la/` (`hsdaoh_la.c`,
`pico_serial.c/.h`, `README.md`).

**Checked before hardware:** the capture logic against a reference model
(20,000 random captures with triggers and data gaps); the la8 commands
against a fake firmware on a virtual serial port; the driver itself against
a fake libhsdaoh stream (trigger position, contiguous data, Stop, data lost
mid-capture, rate change).

**Confirmed on hardware:** detection, capture, sample rate change over the
Pico's serial port, trigger position slider, and 140 MS/s.

---

## 3. cypress-fx3 driver (commits `219cfe4`, `2a8939f`, `1c04ab0`)

Logic analyzer on a Cypress (Infineon) FX3. Ported from the libsigrok
`cypress-fx3-8` and `cypress-fx3` (16 bit) drivers by Ashwin Nair
(Infineon). Two firmwares, chosen in **Device Options → channel mode**:

| Channel mode | Firmware (`DSView/res`) | Samples | Rates |
| --- | --- | --- | --- |
| Use Channels 0~7 (Max 200MHz) | `cypress-fx3-8.fw` | 1 byte | 500 kS/s – 200 MS/s |
| Use Channels 0~15 (Max 100MHz) | `cypress-fx3-16.fw` | 2 bytes | 500 kS/s – 100 MS/s |

### Firmware upload

ALL LOGIC could only load FX2 firmware. The FX3 loader from libsigrok's
`ezusb.c` (`ezusb_fx3_ram_write`, `ezusb_install_firmware_fx3`,
`ezusb_upload_firmware_fx3`) was added to
`libsigrok4DSL/hardware/common/ezusb.c`, with these changes:

- the image is read from a file path (no `sr_resource_load()` here)
- sections are bounds-checked, so a truncated image cannot make it read
  past the buffer
- the USB handle is closed on error paths
- the result is compared with `SR_OK`: `SR_ERR` is +1 in ALL LOGIC but −1
  in libsigrok, so the original `< 0` test would have reported a failed
  upload as success

A blank FX3 (`04B4:00F3`) gets the firmware of the chosen mode and comes
back as `04B4:1234` (`sigrok` / `cypress-fx3`).

### Choosing 8 or 16 channels

Both firmwares look identical over USB and cannot return to the boot
loader. So:

- the driver records the firmware it loaded, and the chosen mode, in
  `%LOCALAPPDATA%\AllLogic\cypress-fx3.ini`, so it knows what runs even
  after ALL LOGIC is restarted
- choosing the other mode saves it and shows **"Please reconnect the
  device!"**; the next plug-in loads it
- until then captures use the firmware that actually runs, so data is
  never read with the wrong sample size

### Trigger

libsigrok4DSL has no `soft_trigger_logic`. The trigger runs in the shared
capture core (section 4): ALL LOGIC's simple trigger panel and position
slider, on all 16 channels in the 16 channel mode.

Files: `libsigrok4DSL/hardware/cypress-fx3/` (`cypress_fx3.c`,
`README.md`), `libsigrok4DSL/hardware/common/ezusb.c`, both firmwares in
`DSView/res`. Small GUI change: `DSView/pv/dialogs/deviceoptions.cpp`
shows the reconnect message when a channel mode needs a re-plug (with a
new `ds_reset_last_error()` in `lib_main.c`).

**Checked before hardware:** the loader against a fake FX3 boot loader
with the real firmware images (RAM contents byte for byte, entry
`0x40007D48`, corrupt and truncated images rejected); the driver against a
fake FX3 that recognises which image it received and streams 1 or 2-byte
samples (upload and re-enumeration, triggers including on channel 12,
3 million sample captures, Stop, device going silent, wrong firmware
version, mode switch and re-plug, restart without unplugging). These tests
found and fixed two 16-bit buffer offsets and a re-plug that would not
reload the firmware for 5 seconds.

**Confirmed on hardware:** firmware upload and capture with both firmwares.

---

## 4. Shared capture core

`libsigrok4DSL/hardware/common/capture_core.c/.h`, used by hsdaoh-la and
cypress-fx3. For analyzers that stream every sample and leave the trigger
to the PC:

| Function | Role |
| --- | --- |
| `hl_build_trigger()` | Lookup tables from the trigger conditions: one 64K table per byte of the sample (channels 0–7, and 8–15 when used) |
| `find_trigger()` | Trigger search, one lookup per byte per sample, edges across buffer boundaries |
| `hl_feed()` | Input with a stream counter (hsdaoh-la): detects lost data |
| `hl_feed_bytes()` | Input without a counter (cypress-fx3); joins 16-bit samples split across USB transfers |
| `process()`, `start_capture()` | Pre-trigger history and capture buffer |
| `hl_to_cross()` | Conversion to `LA_CROSS_DATA`, 8 or 16 channels |

It has no dependency on ALL LOGIC or a device library, so it is tested on
its own: 30,000 random 8 and 16-bit captures against a reference model,
and the conversion against a naive bit-by-bit version.

---

## 5. Windows build

There are no prebuilt releases of the Sipeed fork, so the Windows builds
were cross-compiled on Linux with llvm-mingw: Qt 5.15.19 (qtbase,
qtwinextras, qtsvg), GLib 2.86.1, libusb 1.0.29, libuvc and libhsdaoh,
and embedded CPython 3.13 for the protocol decoders, packaged as a
portable zip (no installer).

FFTW (used only by the DSCope spectrum view) could not be downloaded on
the build machine; that build uses a small FFTW-compatible stand-in on
KissFFT. This affects only those builds, not the source in this repository.

---

## 6. Open items

- **FX2 loader error check:** `ezusb_upload_firmware()` in
  `libsigrok4DSL/hardware/common/ezusb.c` (line 136) tests
  `ezusb_install_firmware(...) < 0`, but that function returns `SR_ERR`
  (+1) on failure, so a failed FX2 upload is reported as success. One-line
  fix: compare with `!= SR_OK`. Not changed, as it is Sipeed's code.
- **Display at the end of a capture:** hsdaoh-la and cypress-fx3 show the
  data when the capture is complete (as their libsigrok drivers do).
  Streaming the post-trigger data into the view as it arrives would help
  long captures at slow rates.
- **Multi-stage triggers:** only ALL LOGIC's single-stage simple trigger
  is supported for these two drivers; libsigrok's `soft_trigger_logic`
  supports several stages.
- **SLogic speed-ups** were benchmarked on the build machine, not timed on
  a real SLogic capture.

## Licence

GPLv3 or later, like the rest of ALL LOGIC. The ported drivers keep the
copyright of their libsigrok authors.
