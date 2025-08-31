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

See [docs](./docs) and [examples](./examples).

Note that this package uses [ETL](https://www.etlcpp.com/) but does not pin a
specific version, to avoid conflicts with your application. Set a specific
dependency version in your application to keep the configuration stable.


## References

Other projects with USB PD support:

- [Google Embedded Controller](https://chromium.googlesource.com/chromiumos/platform/ec)
- [Zephyr Project](https://github.com/zephyrproject-rtos/zephyr)
- [USB Power Delivery for Arduino](https://github.com/manuelbl/usb-pd-arduino)
- [usb-pd](https://github.com/Ralim/usb-pd)

Documentation:

- [USB Power Delivery](https://usb.org/document-library/usb-power-delivery)
