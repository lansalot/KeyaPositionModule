# Keya Position Module (Danfoss Mode 1)

This module extracts the Keya encoder wheel-position sensing path used when Danfoss mode is enabled.

Files:
- KeyaPositionModule.h
- KeyaPositionModule.cpp
- KeyaPositionWeb.h (optional)
- KeyaPositionWeb.cpp (optional)

## What Is Included

- Auto-zero data model: AutoZeroParams
- Keya encoder runtime update from heartbeat ticks (uint16 wrap-safe)
- Angle computation from encoder ticks and zero offset
- Auto-zero update engine with:
  - First-zero establish
  - Rapid mode (guidance off, direct zero jump)
  - Precise mode (guidance on, beta soft correction)
- EEPROM helpers for:
  - Auto-zero params
  - keyaTicksPerDeg calibration
- Optional web/UI helpers for:
  - EMA web filter params
  - Web POST parsing helper for shared config fields

## Runtime Inputs Required

Provide these values each loop when Danfoss mode is active:
- gpsSpeed [km/h]
- yaw [deg]
- emaGpsHdg [x10 deg]
- steerAngleActual [deg]
- guidanceActive (watchdog < threshold)
- invertWas (bool)
- keyaEncoderRaw (updated from CAN heartbeat)

## Core API Calls

### Setup

- keyaLoadTicksPerDeg(EEPROM_ADDR_KEYA_TICKS, keyaTicksPerDeg, KEYA_TICKS_PER_DEG_DEFAULT)
- keyaLoadAutoZeroParams(EEPROM_ADDR_AZ_PARAMS, azParams)
- keyaInitAutoZeroRuntime(keyaAutoZeroRuntime)

### CAN heartbeat decode

From Keya heartbeat bytes 0-1:
- keyaUpdateEncoderFromHeartbeat(runtime, rawTick, invertDirection)

### Angle calculation

- deltaTicks = keyaComputeDeltaTicks(keyaEncoderRaw, keyaZeroTicks)
- steerAngleActual = keyaComputeAngleDeg(keyaEncoderRaw, keyaZeroTicks, keyaTicksPerDeg, invertWas)

### Auto-zero update per loop

- keyaUpdateAutoZero(input, azParams, keyaAutoZeroConfig, keyaEncoderRaw, keyaTicksPerDeg, keyaZeroTicks, wasZeroDone, keyaAutoZeroRuntime)

After update:
- azCorrAccum = keyaAutoZeroRuntime.corrAccum

### AOG integration hooks

When PGN 252 steerSensorCounts is received:
- keyaApplyAogSteerSensorCounts(steerSensorCounts, KEYA_TICKS_PER_DEG_DEFAULT, keyaTicksPerDeg)
- keyaSaveTicksPerDeg(...)

When forced zero is requested (wasOffset == 0):
- keyaForceZeroAtCurrentEncoder(keyaEncoderRaw, keyaZeroTicks, wasZeroDone, keyaAutoZeroRuntime)

## EEPROM Addresses Used

- 84: keyaTicksPerDeg
- 90: AutoZeroParams
- 150: emaYawAlpha
- 154: emaRollAlpha
- 158: emaPitchAlpha
- 162: emaStopKmh

## Notes

- This extraction is Teensy/Arduino-friendly but the core functions are transport-agnostic.
- Serial menu usage depends only on KeyaPositionModule.* (core).
- Web configuration usage depends on KeyaPositionWeb.* in addition to the core module.
- If you do not need web EMA controls in a target project, omit KeyaPositionWeb.h/KeyaPositionWeb.ino.
