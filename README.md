# pdsink

> Sink-only USB PD 3.2 stack for embedded systems: SPR + EPR, with a core
> free of platform and vendor dependencies.

<img src="./docs/images/intro1.jpg" width="40%">

This library focuses on the most common needs of PD‑powered projects and on
ease of use.

Features:

- USB PD Rev 3.2 with SPR and EPR modes (sink side).
- Platform-agnostic C++ core.
- Reference implementation: FUSB302B + FreeRTOS.
- MIT license.

Not supported:

- Source role (SRC) and related modes (DRP/DFP/FRS/Alt).


## Why

A lot of embedded designs only need one thing: to be a sink and behave
correctly. In practice, available PD stacks usually come with at least one hard
constraint:

- vendor lock-in / NDA / no public sources,
- tied to a specific OS or framework,
- incomplete sink-side feature coverage (e.g., no EPR support or poor spec
  compliance),
- difficult to extend to new TCPC/MCU combinations.

This library addresses these constraints through a sink-focused implementation
and aims to improve adoption in modern embedded projects.


## Project status

- Status: early stage; sink path validated on FUSB302B + FreeRTOS.
- Coverage: SPR/PPS/EPR sink negotiation validated on the current reference
  path.
- Seeking: validation on additional TCPC/MCU combinations.
- Real-world use: integrated into the
  [Reflow Micro Table](https://github.com/puzrin/reflow_micro/tree/master/firmware/src/heater)
  firmware (heater control path).

## Usage

See the [docs](./docs) and [examples](./examples).

- Current reference path: `examples/fusb302_rtos_esp32c3_arduino` (FreeRTOS,
  ESP32-C3, FUSB302).
- For a complex real-world scenario, see the heater-related classes in the
  [Reflow Micro Table](https://github.com/puzrin/reflow_micro/tree/master/firmware/src/heater)
  project.

## When pdsink makes sense

This project can help when:

- you need dynamic power control at runtime (heaters, for example).
- you want to use an MCU with an embedded UCPD and simplify external components.
- you need “non-standard” voltages or current limits (via PPS/AVS profiles).

If you only need a single fixed profile, a simple PD trigger (e.g., CH224 or a
similar external trigger) can be a more rational choice.

Project scope: sink role only. The non-goals in `Not supported` are intentional.

## References

Other projects with USB PD support:

- [Google Embedded Controller](https://chromium.googlesource.com/chromiumos/platform/ec)
- [Zephyr Project](https://github.com/zephyrproject-rtos/zephyr)
- [USB Power Delivery for Arduino](https://github.com/manuelbl/usb-pd-arduino)
- [usb-pd](https://github.com/Ralim/usb-pd)

Documentation:

- [USB Power Delivery](https://usb.org/document-library/usb-power-delivery)
