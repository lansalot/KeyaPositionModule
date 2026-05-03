# Portable Bundles

This folder contains pre-assembled module bundles for reusing Keya Danfoss mode 1 position sensing in other projects.

## Variants

- keya-position-core: sensing + auto-zero core only.
- keya-position-core-web: core plus optional web/menu companion files.

## Quick Copy

1. Copy one bundle folder into your target sketch/project.
2. Include the copied .h/.ino files in that project.
3. Follow KeyaPositionIntegrationTemplate.h for host glue.

## Notes

- The core bundle does not require web UI helpers.
- The core+web bundle includes zWebConfig and zAutoZeroMenu for tuning/persistence flows.
- Module implementation files are provided as .cpp for Arduino preprocessor compatibility.
- Existing host-side externs (gpsSpeed, yaw, emaGpsHdg, watchdog, etc.) are still required.