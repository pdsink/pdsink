USB PD stack usage <!-- omit in toc -->
==================

- [Base](#base)
  - [pd\_config.h](#pd_configh)
  - [Built-in drivers](#built-in-drivers)
- [Customization](#customization)
  - [Drivers](#drivers)
  - [Device Policy Manager](#device-policy-manager)
  - [Logging](#logging)
  - [Event loop](#event-loop)
    - [Project with RTOS support](#project-with-rtos-support)
    - [Project without RTOS](#project-without-rtos)
- [Debugging](#debugging)

<img src="./images/intro2.jpg" width="40%">

This document describes generic principles only. You are expected to explore the
[examples](../examples/) folder before getting started.

## Base

### pd_config.h

You should create `pd_config.h` in a searchable path of your project. It will be
loaded automatically to configure the library. As an alternative - you can set
all variables via the `-D` option, but that's more tedious.

The configuration is used for:

- Selecting built-in drivers
- Providing log mapping and activating desired log levels in modules. See
  examples and [pd_log.h](../src/pd_log.h) for available options.

### Built-in drivers

The library contains some built-in drivers for popular hardware, disabled by
default. See [src/drivers/](../src/drivers/) folder and `src/pd_include.h` for
available options.

## Customization

### Drivers

The most frequent cases can be:

- IO pin changes
- Task stack & priority changes (for RTOS-based drivers)

These can be achieved through class inheritance and by updating properties
in the constructor.

Additionally, you are not forced to use built-in drivers. You can clone them to
modify as needed or even create new ones to support different hardware or RTOS.

### Device Policy Manager

The USB PD specification does not provide any information about DPM
architecture. We provide a simple DPM, suitable for basic operation:

- Automatic PD profile selection, based on desired voltage and current.

You may wish to modify DPM for:
- Advanced PD profile selection strategy
- Alert listening
- Controlling PD handshake status

See `MsgToDpm_*` in [messages.h](../src/messages.h), examples, and the `DPM`
class.

NOTE: When you handle DPM events, handlers should be non-blocking. For heavy
operations, use RTOS events to decouple processing.

### Logging

By default, logging is disabled. To use it:

- Add `jetlog` to project dependencies.
- Create a simple wrapper.
- Set variables in config to enable desired levels and modules.

Please refer to the examples for details.

### Event loop

See the `Task` class. By default, event propagation occurs immediately via a
simple event loop with re-entrance protection. This should be suitable for both
threaded and interrupt contexts.

There are some general considerations:

#### Project with RTOS support

This is the most common case. By default, no action is required.

To minimize code in interrupt handlers, reorganize `Task` to process events in a
separate thread. However, this is likely not needed. See details in the next
section on how to implement such changes.

If you use a driver with a built-in thread (FUSB302, for example), no extra care
is required. The event loop already works in the driver's thread context.

TBD: alternative drivers & RTOSes.

#### Project without RTOS

NOTE: The driver must support this mode (to operate without RTOS dependency).

In most cases, no action is required. The library code is ready to be executed
from an interrupt context. But if you wish to further decouple interrupt
processing, do the following:

- Override the `Task.set_event()` method to remove the loop execution from that
  location.
- Invoke `Task.loop()` manually from an external loop (from `main()`, for
  example). To avoid unnecessary CPU load, sleep until an interrupt arrives
  (via `WFI` or something similar).

## Debugging

If something goes wrong, the first step is to enable logging. See provided
examples on how to do that.

**PD logs**

Start with enabling debug logs in all modules. Then reduce to the desired
level/location.

NOTE: EPR chargers (28v and above) can be noisy with full logs, due to EPR pings
every 0.5 second. If it's not critical for you, use an SPR charger to reduce
the noise.

**ETL assert logs**

This is usually not required, but can help if you develop a driver. They can
show recursive (improper) FSM calls, for example.

**Logger priority and stack size**

This library uses `jetlog`, specially optimized for fast writes from any
context, including interrupts. Log reading and output to console happen in a
background thread, to avoid blocking of important tasks.

In very specific cases, if you suspect a PD thread crash that blocks low
priority tasks, increase logger priority to the same as PD priority. This will
cause the log reader to continue doing its job and show you all log buffer
content before the crash happened.

You are recommended to set the log writer record size to 256 bytes, and have
512 extra stack size in all places where the log writer is invoked. The log
buffer is circular, and 10K is usually a good choice to see enough of the last
messages. These recommendations are not strict, just a starting point. Adjust
for your project if needed.
