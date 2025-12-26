# Changelog

## [Unreleased]

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


## [0.1.0] - 2025-11-06

### Added

- Core USB PD sink stack with SPR (Fixed, PPS) and EPR (28 V+) support, built
  to be platform-agnostic.
- Reference FUSB302 + FreeRTOS driver and ESP32-C3 PlatformIO/Arduino example,
  with a simple Device Policy Manager for automatic profile selection.
- Configurable logging (jetlog), `pd_config.h` build-time configuration, and
  starter documentation/tests.

[Unreleased]: https://github.com/pdsink/pdsink/compare/0.1.0...HEAD
[0.1.0]: https://github.com/pdsink/pdsink/releases/tag/0.1.0
