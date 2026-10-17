# cypress-fx3: Cypress FX3 logic analyzer in ALL LOGIC

Logic analyzer on a Cypress (Infineon) FX3 (CYUSB301x): the FX3's GPIF
samples its inputs and streams them over USB 3 bulk endpoint 0x82 (PIB
clock 400 MHz / integer sampling factor). Two firmwares, chosen in the
device options as the channel mode:

| Channel mode | Firmware (`DSView/res`) | Samples | Rates |
| --- | --- | --- | --- |
| Use Channels 0~7 (Max 200MHz) | `cypress-fx3-8.fw` | 1 byte | 500 kS/s .. 200 MS/s |
| Use Channels 0~15 (Max 100MHz) | `cypress-fx3-16.fw` | 2 bytes, little endian | 500 kS/s .. 100 MS/s |

Port of the libsigrok `cypress-fx3-8` and `cypress-fx3` (16 bit) drivers
(Ashwin Nair, Infineon). `cypress-fx3-16.fw` is the 16-bit driver's
`cypress-fx3.fw`, renamed so the two are told apart.

## Firmware loading

The firmware lives in the FX3's RAM and is loaded at every plug-in:

- an FX3 without firmware enumerates with its ROM boot loader
  **04B4:00F3** (or a second-stage boot loader as **04B4:1234** with other
  USB strings)
- the driver loads the firmware of the chosen channel mode with
  `ezusb_upload_firmware_fx3()` (`../common/ezusb.c`, the libsigrok
  loader: 0xA0 writes in 4 KB chunks, checksum, jump to the entry)
- the FX3 re-enumerates as **04B4:1234**, `sigrok` / `cypress-fx3`, and
  shows up as **Cypress FX3**

Both firmwares enumerate exactly alike, so which one runs cannot be read
from the device. The driver records the firmware it loaded, with the chosen
mode, in `<user config dir>/AllLogic/cypress-fx3.ini` (`channel_mode`,
`loaded_firmware`), so it also knows after ALL LOGIC is restarted.

## Switching between 8 and 16 channels

Neither firmware has a command to return to the boot loader, so a running
FX3 cannot be switched in place:

1. choose the other channel mode in the device options: the choice is
   saved and ALL LOGIC shows **"Please reconnect the device!"**
2. unplug and plug in the FX3: the chosen firmware is loaded

Until then captures keep using the firmware that runs (its channels and
rates), so data is never read with the wrong sample width. If the FX3
already runs a firmware ALL LOGIC did not load (e.g. loaded by PulseView),
the driver assumes the chosen mode and asks for a reconnect when a mode is
chosen.

## Using it

- **Windows:** both the boot loader (04B4:00F3) and the running firmware
  (04B4:1234) need the WinUSB driver (Zadig: Options > List All Devices).
  The second one appears only after the first upload, so run Zadig again
  then.
- Use a USB 3 port: on USB 2 (about 40 MB/s) the high rates lose data; the
  log warns when the FX3 is not on a SuperSpeed link.
- **Trigger:** the simple trigger (0, 1, rising, falling, either edge per
  channel, channels 8~15 too in the 16 channel mode), applied on the PC;
  the trigger position slider sets the share of samples before the
  trigger. The capture is collected in RAM and shown when it is complete;
  **Stop** shows what was captured after the trigger.
- If the FX3 stops sending (two empty or failed transfers in a row, as in
  the libsigrok drivers), the acquisition ends with what was captured.

## Protocol (vendor requests on endpoint 0, both firmwares)

| Request | Dir | Data |
| --- | --- | --- |
| 0xB0 GET_FW_VERSION | IN | major, minor (major must be 1) |
| 0xB2 GET_REVID | IN | 1 byte |
| 0xB1 START | OUT | u16 little endian: 400 MHz / sample rate |
