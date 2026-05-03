# Keya Position Core Bundle

Contents:
- KeyaPositionModule.h
- KeyaPositionModule.cpp
- KeyaPositionIntegrationTemplate.h

Use this bundle when you only need:
- Keya encoder angle sensing for Danfoss mode 1
- Auto-zero runtime logic
- EEPROM load/save helpers for auto-zero params and ticks-per-degree

Host project still needs to provide runtime inputs such as gpsSpeed, yaw, emaGpsHdg, guidance state, and encoder heartbeat ticks.

Integration steps:
1. Include KeyaPositionModule.h in host files using the module.
2. Load config at startup with keyaLoadTicksPerDeg and keyaLoadAutoZeroParams.
3. Update encoder from CAN heartbeat using keyaUpdateEncoderFromHeartbeat.
4. Compute angle with keyaComputeAngleDeg.
5. Run keyaUpdateAutoZero in timed loop.
6. Use KeyaPositionIntegrationTemplate.h as the glue checklist.