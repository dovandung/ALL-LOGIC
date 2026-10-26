# alientek-dl32: ALIENTEK DL32 / DL32 Plus in ALL LOGIC

Port of the libsigrok `alientek-dl32` driver
([dovandung/libsigrok](https://github.com/dovandung/libsigrok), branch
`master`, up to `96aad41`) to ALL LOGIC's libsigrok4DSL. The protocol code
(`dl32_protocol.c`) is the libsigrok code with only the host glue changed;
`dl32.c` is the ALL LOGIC driver around it.

| Model | Channels | Stream Mode | Buffer Mode |
| --- | --- | --- | --- |
| DL32 Plus | 32 | 1 - 100 MHz | 1 MHz - 1 GHz, rate x channels <= 8 G |
| DL32 | 16 | 1 - 100 MHz | 1 MHz - 1 GHz, rate x channels <= 4 G |

The model and channel count come from the device's hello reply (USB ID
04B4:6A6A, which is a Cypress FX3 ID, so the device is confirmed with the
DL32 hello exchange before it is listed).

## Using it

- **Windows:** the DL32 needs the WinUSB driver (Zadig: Options > List All
  Devices). With the Cypress cyusb3 driver installed by ATK-Logic, the log
  says so. Close ATK-Logic first.
- **Stream Mode / Buffer Mode** in Device Options. Rates above 100 MHz are
  offered in Buffer Mode only. In Buffer Mode the device captures into its
  memory and the capture is shown when it is complete; Start is refused
  when rate x enabled channels exceeds the budget (the log names the
  largest rate or channel count that fits). ALL LOGIC's sample depth uses
  ATK-Logic's DL16 memory figures (1 G samples x channels, 3.5 G on the
  Plus), assumed for the DL32.
- **Trigger:** the simple trigger (0, 1, rising, falling, either edge per
  channel, all at once) is the device's own hardware trigger. The trigger
  position slider sets the pre-trigger part. In Buffer Mode the window is
  trimmed so the trigger sits at that point; in Stream Mode the trigger
  marker is put where the device reports it fired.
- **Threshold (VTH):** 0 to 5 V in 0.1 V steps (the device takes 0.1 V
  units). ATK-Logic's presets: 5.0 V CMOS 2.5 V, 3.3 V CMOS 1.6 V (the
  default), 3.0 V CMOS 1.5 V, 2.5 V CMOS 1.2 V, 1.8 V CMOS 0.9 V, 1.5 V
  CMOS 0.7 V, 1.2 V CMOS 0.6 V.
- **RLE (Buffer Mode):** handled the way ALL LOGIC does it for DSLogic.
  With Buffer Mode selected, the sample count list goes past the device
  memory with entries marked "(RLE)"; choosing one turns RLE on, and the
  device then sends run-length coded data (pairs of count and byte). If the
  coded data still does not fit, the device reports it, the channels are
  aligned on the shortest one and ALL LOGIC is told the actual length (the
  log says how many samples fit). ATK-Logic offers RLE up to 10 times the
  memory; ALL LOGIC's list goes further, so the longest RLE counts may end
  early. RLE is not used in Stream Mode, where ALL LOGIC's sample count
  list never selects it.
- **Environment:** `ALIENTEK_DL32_OFFSET=<n>` shows a 16 channel window
  from CH`n` (even, at most 16); `ALIENTEK_DL32_DEBUG=1` adds hex dumps of
  the commands and frames to the log.

## Data

The device sends one bit stream per channel, 8 samples per byte, LSB first.
That is already ALL LOGIC's LA_CROSS_DATA plane layout, so stream data is
copied 8 bytes per channel and 64 samples (no interleaving), and Buffer
Mode data is cut out of each channel's stream at the trimmed start.

## Not ported

- RLE in Stream Mode (the libsigrok driver allows it; ALL LOGIC's sample
  count list turns RLE off whenever Stream Mode is used)
- libsigrok's software trigger fallback (ALL LOGIC's simple trigger always
  fits the hardware trigger)
- the `ALIENTEK_DL32_DUMP` raw dump file
