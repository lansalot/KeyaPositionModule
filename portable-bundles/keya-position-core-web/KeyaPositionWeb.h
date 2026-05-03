#ifndef KEYA_POSITION_WEB_H
#define KEYA_POSITION_WEB_H

#include "KeyaPositionModule.h"

class String;

struct KeyaWebEmaParams
{
  float yawAlpha;
  float rollAlpha;
  float pitchAlpha;
  float stopKmh;
};

struct KeyaWebApplyResult
{
  bool azUpdated;
  bool ticksUpdated;
  bool emaUpdated;
};

KeyaWebApplyResult keyaApplyWebPostValues(const String &body,
                                          AutoZeroParams &params,
                                          float &ticksPerDeg,
                                          KeyaWebEmaParams &ema);

void keyaLoadWebEmaParams(int eepromAddrYaw,
                          int eepromAddrRoll,
                          int eepromAddrPitch,
                          int eepromAddrStop,
                          KeyaWebEmaParams &ema);

void keyaSaveWebEmaParams(int eepromAddrYaw,
                          int eepromAddrRoll,
                          int eepromAddrPitch,
                          int eepromAddrStop,
                          const KeyaWebEmaParams &ema);

#endif