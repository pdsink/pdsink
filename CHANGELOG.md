# Changelog

## Unreleased

- Driver: Refactored the API for new and future needs.
- Timers: Switched to microsecond resolution.
- TC: Added support for hardware polarity detection.
- PE: Added support for `Get_Sink_Cap_Extended`.
- PE: Updated the `Revision` message info to USB PD 3.2 v1.2.
- PE: Wait for VBUS before starting the Source Capabilities timeout.
- PE: Relaxed Source Capabilities validation to allow a fixed 28 V PDO in SPR
  mode (FMchip chargers).
- PE: Preserve the negotiated PD revision across capability updates
  and Soft Reset.
- PE: Omit APDOs from Sink Capabilities sent to PD 2.0 sources.
- PE: Moved the start of the EPR entry timer from queuing the request to
- receiving GoodCRC.
- PE: Send a Soft Reset when EPR mode entry fails.
- PE: Send a Hard Reset if the source rejects the first `EPR_Request`.
- PE: Corrected the BIST Carrier Mode duration from 300 ms to 45 ms.
- PE/PRL: Respect `SinkTxOK` for periodic PPS requests, EPR keepalives and
  retries after `Wait`.
- PRL: Skip the `SinkTxOK` wait for PD 2.0 sources.
- PRL: Fixed a stalled `Accept` reply when a Soft Reset interrupted
  a sink-initiated exchange.
- PRL: Fixed MessageID tracking across discarded transmissions and resets.
- PRL: Fixed padding of outgoing extended message chunks.
- PRL: Reject truncated or mismatched chunks in extended messages.
- PRL: Report interrupted Chunk Requests.
- DPM: Wake the driver task as soon as the application requests a new power level.
- DPM: Fixed missing voltage and current limits in fixed sink PDOs.
- DPM: Clear the reserved EPR Mode Capable bit in PD 2.0 power requests.
- DPM: Fixed a power-to-current conversion that could request zero current for EPR AVS.
- FUSB302: Refactored the architecture for RX/TX eventual consistency.
- FUSB302: Switched to asynchronous RX with a single-packet buffer.
- FUSB302: Replaced periodic timer ticks with a one-shot timer.
- FUSB302: Use hardware CC toggling to detect cable orientation.
- FUSB302: Implement packet deduplication and filtering on the driver side.
- FUSB302: Add extra silence checks before TX to minimize the collision window.
- Reduced the flash footprint of state machines and logging.
- Simplified jetlog setup to a single `PD_JETLOG_FN` macro.


## 0.1.2 - 2026-09-08

- Added jetlog v2 optimizations.
- Examples polish.


## 0.1.1 - 2026-01-19

### Added

- Added Source_Capabilities validator. Nothing serious, just to follow the
  spec.

### Changed

- ESP32 FUSB302 HAL: rolled back to the "legacy" I2C API. The new one has a
  serious unfixed bug: https://github.com/espressif/esp-idf/issues/14030. The
  new API code was removed for now until it becomes more robust.
- Enabled hardware filter for SDA/SCL lines; it had been missing in the old API
  code.
- Allowed EPR_Source_Capabilities in PE_SNK_Ready state while in SPR mode. The
  spec is unclear about this case.
- Polished documentation and added a link to a real-world usage example (Reflow
  Micro Table project).

### Fixed

- Soft Reset from a port partner was broken and caused a hard reset. PE tried to
  send `Accept` before PRL finished its job, causing a race condition.
- FUSB302 driver improperly reported support of hardware auto-retries. That
  caused extra software retries from PRL.
- Fixed TCH_Wait_Chunk_Request_State first-timeout reaction (now follows the
  spec and reports as "success" for compatibility with other PD
  implementations).
- Clarified the hard reset counter check condition (was 1 less than required).
- Fixed the target state after exiting PE_BIST_Carrier_Mode according to the
  spec.
- Fixed deferred wakeups in PE/PRL. Nothing happened until the next timer tick,
  instead of immediate event loop wakup.


## 0.1.0 - 2025-11-06

### Added

- Core USB PD sink stack with SPR (Fixed, PPS) and EPR (28 V+) support, built
  to be platform-agnostic.
- Reference FUSB302 + FreeRTOS driver and ESP32-C3 PlatformIO/Arduino example,
  with a simple Device Policy Manager for automatic profile selection.
- Configurable logging (jetlog), `pd_config.h` build-time configuration, and
  starter documentation/tests.
