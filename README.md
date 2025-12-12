# pdsink

> USB PD Sink library for embedded devices.

<img src="./docs/images/intro1.jpg" width="40%">

This library focuses on the most common needs of PD‑powered projects and on
ease of use.

Features:

- SPR modes (Fixed and PPS).
- EPR modes (28 V and up).
- No platform dependencies in the core.

Not supported:

- Source role (SRC).
- DRP / DFP / FRS / Alt Modes.


## Usage

See the [docs](./docs) and [examples](./examples).

For a real-world complex scenario, see the heater-related classes in the
[Reflow Micro Table](https://github.com/puzrin/reflow_micro/tree/master/firmware/src/heater)
project.

This package uses [ETL](https://www.etlcpp.com/) but leaves the version
unpinned to avoid conflicts with your application. Pin ETL in your project to
keep the configuration stable.


## When pdsink makes sense

This project can help when:

- you need dynamic power control at runtime (heaters, for example).
- you want to use an MCU with an embedded UCPD and simplify external components.
- you need “non-standard” voltages or current limits (via PPS/AVS profiles).

If you only need a single fixed profile, a simple PD trigger (e.g., CH224 or an
external one) can be a more rational choice.


## References

Other projects with USB PD support:

- [Google Embedded Controller](https://chromium.googlesource.com/chromiumos/platform/ec)
- [Zephyr Project](https://github.com/zephyrproject-rtos/zephyr)
- [USB Power Delivery for Arduino](https://github.com/manuelbl/usb-pd-arduino)
- [usb-pd](https://github.com/Ralim/usb-pd)

Documentation:

- [USB Power Delivery](https://usb.org/document-library/usb-power-delivery)
