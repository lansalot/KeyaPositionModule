#ifndef KEYA_POSITION_INTEGRATION_TEMPLATE_H
#define KEYA_POSITION_INTEGRATION_TEMPLATE_H

/*
  Copy/Paste integration template for projects that want only:
  - Danfoss mode 1 Keya position sensing
  - Auto-zero behavior
  - zAutoZeroMenu + zWebConfig persistence/tuning support

  Required files to copy:
  - KeyaPositionModule.h
  - KeyaPositionModule.ino
  - KeyaPositionWeb.h + KeyaPositionWeb.ino (optional, only for zWebConfig support)
  - zAutoZeroMenu.ino (optional if no serial tuning needed)
  - zWebConfig.ino (optional if no web tuning needed)

  Required project globals (host app):
  - float gpsSpeed;
  - float yaw;
  - float emaGpsHdg;              // x10 deg
  - float steerAngleActual;
  - float keyaTicksPerDeg;
  - int32_t keyaZeroTicks;
  - bool wasZeroDone;
  - int32_t keyaEncoderRaw;
  - AutoZeroParams azParams;
  - uint8_t watchdogTimer;

  Typical host state:
  - KeyaAutoZeroRuntime keyaAutoZeroRuntime;
  - KeyaAutoZeroConfig keyaAutoZeroConfig = keyaDefaultAutoZeroConfig();
  - float azCorrAccum = 0.0f;

  Setup sequence:
  1) keyaLoadTicksPerDeg(EEPROM_ADDR_KEYA_TICKS, keyaTicksPerDeg, KEYA_TICKS_PER_DEG_DEFAULT);
  2) keyaLoadAutoZeroParams(EEPROM_ADDR_AZ_PARAMS, azParams);
  3) keyaInitAutoZeroRuntime(keyaAutoZeroRuntime);

  CAN heartbeat decode (ID 0x07000001 bytes 0-1):
  -------------------------------------------------
  // bool invertEncoderDir = true if wiring/motor direction needs inversion
  KeyaEncoderRuntime encRt = {keyaEncoderRaw, keyaEncPrev, keyaEncInitDone};
  keyaUpdateEncoderFromHeartbeat(encRt, encTick, invertEncoderDir);
  keyaEncoderRaw = encRt.encoderRaw;
  keyaEncPrev = encRt.prevTick;
  keyaEncInitDone = encRt.initDone;

  Timed loop integration:
  -------------------------------------------------
  int32_t deltaTicks = keyaComputeDeltaTicks(keyaEncoderRaw, keyaZeroTicks);
  steerAngleActual = keyaComputeAngleDeg(keyaEncoderRaw,
                                         keyaZeroTicks,
                                         keyaTicksPerDeg,
                                         invertWas);

  KeyaAutoZeroInput azIn = {
      .nowMs = millis(),
      .gpsSpeedKmh = gpsSpeed,
      .yawDeg = yaw,
      .emaGpsHeadingX10Deg = emaGpsHdg,
      .steerAngleActualDeg = steerAngleActual,
      .invertWas = invertWas,
      .guidanceActive = (watchdogTimer < WATCHDOG_THRESHOLD)};

  bool azCommitted = keyaUpdateAutoZero(azIn,
                                        azParams,
                                        keyaAutoZeroConfig,
                                        keyaEncoderRaw,
                                        keyaTicksPerDeg,
                                        keyaZeroTicks,
                                        wasZeroDone,
                                        keyaAutoZeroRuntime);

  azCorrAccum = keyaAutoZeroRuntime.corrAccum;

  AOG config hooks:
  -------------------------------------------------
  // On steerSensorCounts update:
  keyaApplyAogSteerSensorCounts(steerSensorCounts,
                                KEYA_TICKS_PER_DEG_DEFAULT,
                                keyaTicksPerDeg);
  keyaSaveTicksPerDeg(EEPROM_ADDR_KEYA_TICKS,
                      keyaTicksPerDeg,
                      KEYA_TICKS_PER_DEG_DEFAULT);

  // On forced zero command (wasOffset == 0):
  keyaForceZeroAtCurrentEncoder(keyaEncoderRaw,
                                keyaZeroTicks,
                                wasZeroDone,
                                keyaAutoZeroRuntime);
*/

#endif