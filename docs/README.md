USB PD stack usage <!-- omit in toc -->
==================

- [Base](#base)
- [Customization](#customization)
  - [Drivers](#drivers)
  - [Device Policy Manager](#device-policy-manager)
  - [Logging](#logging)
  - [Event loop](#event-loop)
- [Debugging](#debugging)

<img src="./images/intro2.jpg" width="40%">

This document covers the basics. **Check the [examples](../examples/) folder
before getting started.**

## Base

> **Note.** This package uses [ETL](https://www.etlcpp.com/) but leaves the
> version unpinned to avoid conflicts with your application. Pin ETL in your
> project to keep the configuration stable.

**-D PD_USE_CONFIG_FILE**

Create a `pd_config.h` file in a searchable path in your project.
The library will load it to configure itself. Alternatively, you can set all
variables via the `-D` option, but that is more tedious when logging is
enabled.

**Built-in drivers**

The library contains built-in drivers for popular hardware, which are disabled
by default. See the [drivers](../src/pd/drivers/) folder,
[pd_include.h](../src/pd/pd_include.h) and [examples](../examples/) for
available options.

**For PlatformIO**

Application `platformio.ini` build options:

```
lib_deps =
  https://github.com/pdsink/pdsink#<tag_or_hash>

build_flags =
  -I $PROJECT_DIR/<path_to_config_folder>
  -D PD_USE_CONFIG_FILE
```

**For other build systems**

- Add the pdsink `src/` folder to the project search path.
- If logs are enabled, add the `jetlog` path to the pdsink library build options.
- If you are using `pd_config.h`, make its folder searchable for both the project
  and the library.


## Customization

### Drivers

The most common cases are:

- IO pin changes
- Task stack & priority changes (for RTOS-based drivers)

You can handle these cases through class inheritance and by updating
constructor properties.

You do not have to use the built-in drivers as they are. You can clone and
tweak them, or write your own to support different hardware or RTOS.
Contributions are welcome.

### Device Policy Manager

The USB PD specification does not provide any information about the DPM
architecture. We provide a simple DPM suitable for basic operation:

- Automatic PD profile selection based on the desired voltage and current.

You may wish to modify the DPM for:
- advanced PD profile selection strategies,
- alert handling,
- PD handshake status control.

See `MsgToDpm_*` in [messages.h](../src/pd/messages.h), examples, and the `DPM`
class.

Note: When handling DPM events, keep handlers non-blocking. For heavy
operations, use RTOS events to decouple processing.

### Logging

By default, logging is disabled. To use it:

- Add `jetlog` to project dependencies.
- Create a simple wrapper (see examples).
- Set configuration variables to enable the desired levels and modules.
- For PlatformIO, if logging is enabled and you see `jetlog.hpp` include errors,
  add `-D PD_USE_JETLOG` to the build flags.

See the examples for details.

### Event loop

See the `Task` class. By default, event propagation occurs immediately via a
simple event loop with reentrancy protection. This should be suitable for both
threaded and interrupt contexts.

You may wish to decouple events from interrupts in several cases:

1. If you use a driver without RTOS, but want to run the PD stack in a high
   priority task.
2. If your platform runs without RTOS at all, and you want to call the
   dispatcher from the application main loop.

In this case, override `Task.set_event()` and `Task.dispatch()` to suit your
needs.

Note: This is not required if the driver already has an RTOS task inside
(for example, FUSB302).

## Debugging

If something goes wrong, the first step is to enable logging. See the provided
examples on how to do that.

**PD logs**

Start by enabling debug logs in all modules. Then reduce them to the desired
level or location.

Note: EPR chargers (28 V and above) can produce noisy full logs due to EPR
pings every 0.5 s. If that is not critical for you, use an SPR charger to reduce
the noise.

**ETL assert logs**

This is usually not required, but it can help when you develop a driver. These
logs can show recursive (improper) FSM calls, for example.

**Logger priority and stack size**

This library uses `jetlog`, which is specifically optimized for fast writes from
any context, including interrupts. Log reading and output to the console happen
in a background thread to avoid blocking important tasks.

In very specific cases, if you suspect a PD thread crash that blocks
low-priority tasks, increase the logger priority to match the PD priority. This
will allow the log reader to continue doing its job and show all log buffer
content before the crash happens.

Recommended: set the log-writer record size to 256 bytes, and add an extra
512 bytes of stack in all places where the log writer is invoked. The log
buffer is circular; 10 KB is usually a good choice to keep enough recent
messages.

For the log reader, set the message buffer to 256 bytes. If batching is
implemented, use a batch size of 1–4 KB. Batching helps with burst writes:
sending fewer large blocks via USB VCOM is much faster than many small ones.

These recommendations are not strict; they are just a starting point. Adjust
them for your project if needed.
