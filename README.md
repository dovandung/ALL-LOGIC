# ALL LOGIC


I using AI CLAUDE to convert libsigrok of FX3 , HSDAOH to ALLLOGIC driver and it works ok
ALL LOGIC is an **unofficial** multi-vendor logic analyzer host.

This project is **not a from-scratch rewrite**. It is **secondary development** on the already open-sourced **[DSView](https://github.com/DreamSourceLab/DSView)** code from DreamSourceLab. DSView itself is based on [sigrok](https://sigrok.org) PulseView. Original copyright notices remain in the corresponding source files.

**ALL LOGIC is not an official DreamSourceLab product and is not licensed or endorsed by any hardware vendor.**  
Names such as “DSLogic” in the device list are product names reported by the instrument. They do not mean official vendor support.

### What we changed

On top of the original DSView sources we mainly:

- Rebranded the host as ALL LOGIC (name and icon)
- Added community drivers for additional logic analyzers (interoperability)
- Added a local MCP interface so AI clients can control capture and decode
- Replaced the About / legal text so the secondary-modification status is explicit

### Supported devices (community drivers)

These were added in this secondary development. They are **not** official vendor hosts:

1. **CH32H417 logic analyzer** — [OSHWHUB project](https://oshwhub.com/q2h2/project_bszkxrnf) · [firmware](https://github.com/Doukeyi-X/OpenSourceLogic-CH32H417)
2. **Sipeed SLogic Combo 8, SLogic16U3, and SLogic32U3** — [Sipeed introduction](https://wiki.sipeed.com/hardware/zh/logic_analyzer/slogic16u3/Introduction.html)
3. **PXLogic32U3 (5 Gbps)** — [MarryChip](https://marrychip.com/)
4. **ATK-Logic (Alientek DL16 etc.)** — [official host source](https://github.com/alientek-openedv/atk-logic)
5. **nanoDLA / Cypress FX2 (fx2lafw)** — [wuxx/nanoDLA](https://github.com/wuxx/nanoDLA)
6. FX3 Cypress
7. HSDAOH-RP2350

DreamSourceLab instruments already supported by upstream DSView still work.

See [CHANGELOG.md](CHANGELOG.md). Current version is **1.4.0**.

CH32H417 **first use**: flash IAP → USB CDC enumerates → ALL LOGIC **Firmware Upgrade** writes APP → after the first APP, install WinUSB with Zadig → unplug/replug → open ALL LOGIC and capture. Full steps: [firmware repo](https://github.com/Doukeyi-X/OpenSourceLogic-CH32H417).

### Star, bugs, and new hardware

If ALL LOGIC helps you, please **Star** the repo. It makes the project easier to find and keeps the work going.

Found a crash, a device that will not enumerate, or a logic analyzer you want added? Open an issue:

https://github.com/Doukeyi-X/ALL-LOGIC/issues

Please include the model, OS, steps to reproduce, or a hardware / shop link. We will evaluate new analyzers as community interoperability work.

This repository publishes **host source only**. CH32H417 firmware is in [OpenSourceLogic-CH32H417](https://github.com/Doukeyi-X/OpenSourceLogic-CH32H417). Windows setup and portable zip are in [Releases](https://github.com/Doukeyi-X/ALL-LOGIC/releases).

### Layout

| Path | Contents |
|------|----------|
| `DSView/` | GUI (based on original DSView / PulseView) |
| `libsigrok4DSL/` | Device layer and community drivers |
| `libsigrokdecode4DSL/` | Protocol decoders |
| `lang/` | UI language files |
| `tools/package_windows.ps1` | Windows packaging script |
| `COPYING` | Full GNU GPLv3 text |
| `NOTICE.txt` | Secondary-development and trademark notes |

### License

The program as a whole is licensed under the **GNU GPLv3 or later**. See `COPYING` and `NOTICE.txt`.

You may use, modify, and redistribute it. If you distribute binaries, you must also provide the corresponding source and keep this license and the original copyrights.

### Contributors

| | |
|---|---|
| [Doukeyi-X](https://github.com/Doukeyi-X) | ALL LOGIC secondary development |
| [DreamSourceLab](https://github.com/DreamSourceLab) | Original DSView |

### Upstream

- DSView: https://github.com/DreamSourceLab/DSView
- sigrok / PulseView: https://sigrok.org
