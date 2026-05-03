# Keya Position Core + Web Bundle

Contents:
- KeyaPositionModule.h
- KeyaPositionModule.cpp
- KeyaPositionWeb.h
- KeyaPositionWeb.cpp
- KeyaPositionIntegrationTemplate.h
- zAutoZeroMenu.ino
- zWebConfig.ino

Use this bundle when you need full behavior:
- Core Keya sensing + auto-zero runtime
- Serial Auto-Zero menu tuning
- Web config save/load and EMA/UI helper flows

Integration notes:
1. Include both KeyaPositionModule and KeyaPositionWeb files.
2. Keep zWebConfig include set to KeyaPositionWeb.h.
3. Keep EEPROM addresses aligned with the host project:
   - 84 keyaTicksPerDeg
   - 90 AutoZeroParams
   - 150/154/158/162 EMA settings
4. Wire host extern variables expected by zWebConfig and zAutoZeroMenu.

If your target has no web UI, use the core-only bundle instead.