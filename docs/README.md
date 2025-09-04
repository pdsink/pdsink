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

**-D PD_USE_CONFIG_FILE**

Create `pd_config.h` file in a searchable path of your project.
It will be loaded to configure the library. Alternatively, you can set
all variables via the `-D` option, but that's more tedious if you use logging.

**Built-in drivers**

The library contains some built-in drivers for popular hardware, which are
disabled by default. See the [drivers](../src/pd/drivers/) folder,
[pd_include.h](../src/pd/pd_include.h) and [examples](../examples/) for
available options.

**For Platform IO**

Application `platformio.ini` build options:

```
lib_deps =
  https://github.com/pdsink/pdsink#<tag_or_hash>

build_flags =
  -I $PROJECT_DIR/<path_to_config_folder>
  -D PD_USE_CONFIG_FILE
```

**For other build systems**

- For the project, add pdsink `src/` folder to searchable path.
- If logs enabled, add `jetlog` path to pdsink library build options.
- I using `pd_config.h` - make it's foldder searchable for the project and the
  library.


## Customization

### Drivers

The most common cases are:

- IO pin changes
- Task stack & priority changes (for RTOS-based drivers)

These can be achieved through class inheritance and by updating properties
in the constructor.

You don’t have to use the built-in drivers as they are. You can clone and tweak
them, or write your own to support different hardware or RTOS. Contributions are
welcome.

### Device Policy Manager

The USB PD specification does not provide any information about the DPM
architecture. We provide a simple DPM, suitable for basic operation:

- Automatic PD profile selection, based on desired voltage and current.

You may wish to modify the DPM for:
- Advanced PD profile selection strategies
- Alert listening
- Controlling PD handshake status

See `MsgToDpm_*` in [messages.h](../src/pd/messages.h), examples, and the `DPM`
class.

NOTE: When you handle DPM events, handlers should be non-blocking. For heavy
operations, use RTOS events to decouple processing.

### Logging

By default, logging is disabled. To use it:

- Add `jetlog` to project dependencies.
- Create a simple wrapper (see examples).
- Set variables in the config to enable desired levels and modules.
- For PlatformIO, with enabled logging, if you see `jetlog.hpp` include errors -
  add `-D PD_USE_JETLOG` to build flags.

See the examples for details.

### Event loop

See the `Task` class. By default, event propagation occurs immediately via a
simple event loop with reentrancy protection. This should be suitable for both
threaded and interrupt contexts.

You may wish to decouple events from interrupts in several cases:

1. If you use a driver without RTOS, but wish to run the PD Stack in a high
   priority task.
2. If your platform runs without RTOS at all, and you wish to call the
   dispatcher from the application main loop.

In this case, override `Task.set_event()` and `Task.dispatch()` to suit your
needs.

NOTE: This is not required if the driver already has an RTOS task inside
(FUSB302, for example).

## Debugging

If something goes wrong, the first step is to enable logging. See the provided
examples on how to do that.

**PD logs**

Start by enabling debug logs in all modules. Then reduce to the desired
level/location.

NOTE: EPR chargers (28 V and above) can be noisy with full logs due to EPR pings
every 0.5 s. If that’s not critical for you, use an SPR charger to reduce
the noise.

**ETL assert logs**

This is usually not required, but can help if you develop a driver. They can
show recursive (improper) FSM calls, for example.

**Logger priority and stack size**

This library uses `jetlog`, specifically optimized for fast writes from any
context, including interrupts. Log reading and output to the console happen in a
background thread, to avoid blocking important tasks.

In very specific cases, if you suspect a PD thread crash that blocks low
priority tasks, increase the logger priority to the same as the PD priority.
This will allow the log reader to continue doing its job and show you all log
buffer content before the crash happened.

Recommended: set the log-writer record size to 256 bytes, and have
an extra 512 bytes of stack in all places where the log writer is invoked. The
log buffer is circular; 10 KB is usually a good choice to keep enough recent
messages.

For the log reader, set the message buffer to 256 bytes. If batching is
implemented, use a batch size of 1–4 KB. Batching helps with burst writes:
sending fewer large blocks via USB VCOM is much faster than many small ones.

These recommendations are not strict, just a starting point. Adjust
for your project if needed.
