# cypress-fx3: Cypress FX3 logic analyzer in ALL LOGIC

8 channel logic analyzer on a Cypress (Infineon) FX3 (CYUSB301x): the FX3's
GPIF samples 8 inputs and streams them over USB 3 bulk endpoint 0x82, one
byte per sample, at 500 kS/s to 200 MS/s (PIB clock 400 MHz / integer
sampling factor).

Port of the libsigrok `cypress-fx3-8` driver (Ashwin Nair, Infineon).

## Firmware

`DSView/res/cypress-fx3-8.fw`, an FX3 SDK boot image ("CY", 4 RAM
sections, entry 0x40007D48). It is loaded into the FX3's RAM on every
power-up:

- an FX3 without firmware enumerates with its ROM boot loader
  **04B4:00F3** (or a second-stage boot loader as **04B4:1234** with other
  USB strings)
- the driver writes the image over vendor request 0xA0 in 4 KB chunks,
  checks the image checksum and jumps to the entry address
  (`ezusb_upload_firmware_fx3()` in `../common/ezusb.c`, the libsigrok
  loader)
- the FX3 re-enumerates as **04B4:1234** with the strings `sigrok` /
  `cypress-fx3` and shows up in the device list as **Cypress FX3**

## Using it

- **Windows:** both the boot loader (04B4:00F3) and the running firmware
  (04B4:1234) need the WinUSB driver (Zadig: Options > List All Devices).
  The second one appears only after the first upload, so run Zadig again
  then.
- Use a USB 3 port: on USB 2 (about 40 MB/s) the high rates lose data; the
  log warns when the FX3 is not on a SuperSpeed link.
- **Trigger:** the simple trigger (0, 1, rising, falling, either edge per
  channel), applied on the PC; the trigger position slider sets the share
  of samples before the trigger. The capture is collected in RAM and shown
  when it is complete; **Stop** shows what was captured after the trigger.
- If the FX3 stops sending (two empty or failed transfers in a row, as in
  the libsigrok driver), the acquisition ends with what was captured.

## Protocol (vendor requests on endpoint 0)

| Request | Dir | Data |
| --- | --- | --- |
| 0xB0 GET_FW_VERSION | IN | major, minor (major must be 1) |
| 0xB2 GET_REVID | IN | 1 byte |
| 0xB1 START | OUT | u16 little endian: 400 MHz / sample rate |
