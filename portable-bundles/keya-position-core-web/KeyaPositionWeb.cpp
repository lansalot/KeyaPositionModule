#include "KeyaPositionWeb.h"

#include <Arduino.h>
#include <EEPROM.h>

namespace
{
constexpr uint16_t KEYA_AZ_IDENT = 0xA202;

static float keyaExtractFloat(const String &body, const char *key, float defaultValue)
{
  String token = String(key) + "=";
  int start = body.indexOf(token);
  if (start < 0)
    return defaultValue;

  start += token.length();
  int end = body.indexOf('&', start);
  String value = (end < 0) ? body.substring(start) : body.substring(start, end);
  value.trim();
  return value.toFloat();
}

} // namespace

KeyaWebApplyResult keyaApplyWebPostValues(const String &body,
                                          AutoZeroParams &params,
                                          float &ticksPerDeg,
                                          KeyaWebEmaParams &ema)
{
  KeyaWebApplyResult result = {false, false, false};

  AutoZeroParams next = params;
  next.useBno = body.indexOf("useBno=1") >= 0 ? 1 : 0;
  next.useGps = body.indexOf("useGps=1") >= 0 ? 1 : 0;

  float b = keyaExtractFloat(body, "beta", next.beta);
  if (b >= 0.001f && b <= 1.0f)
    next.beta = b;

  next.speedMin = keyaExtractFloat(body, "speedMin", next.speedMin);
  next.yawRateMax = keyaExtractFloat(body, "yawRateMax", next.yawRateMax);
  next.gpsHdgMax = keyaExtractFloat(body, "gpsHdgMax", next.gpsHdgMax);
  next.timeSlowMs = (uint32_t)keyaExtractFloat(body, "timeSlowMs", (float)next.timeSlowMs);
  next.timeFastMs = (uint32_t)keyaExtractFloat(body, "timeFastMs", (float)next.timeFastMs);
  next.speedSlow = keyaExtractFloat(body, "speedSlow", next.speedSlow);
  next.speedFast = keyaExtractFloat(body, "speedFast", next.speedFast);
  next.ident = KEYA_AZ_IDENT;

  if (keyaIsAutoZeroParamsValid(next))
  {
    params = next;
    result.azUpdated = true;
  }

  float nextTicks = keyaExtractFloat(body, "keyaTicks", ticksPerDeg);
  nextTicks = keyaSanitizeTicksPerDeg(nextTicks, ticksPerDeg);
  if (fabsf(nextTicks - ticksPerDeg) > 0.0001f)
  {
    ticksPerDeg = nextTicks;
    result.ticksUpdated = true;
  }

  KeyaWebEmaParams nextEma = ema;
  float v = keyaExtractFloat(body, "emaYaw", nextEma.yawAlpha);
  if (v >= 0.0f && v <= 1.0f)
    nextEma.yawAlpha = v;
  v = keyaExtractFloat(body, "emaRoll", nextEma.rollAlpha);
  if (v >= 0.0f && v <= 1.0f)
    nextEma.rollAlpha = v;
  v = keyaExtractFloat(body, "emaPitch", nextEma.pitchAlpha);
  if (v >= 0.0f && v <= 1.0f)
    nextEma.pitchAlpha = v;
  v = keyaExtractFloat(body, "emaStop", nextEma.stopKmh);
  if (v >= 0.0f && v <= 20.0f)
    nextEma.stopKmh = v;

  if (fabsf(nextEma.yawAlpha - ema.yawAlpha) > 0.0001f ||
      fabsf(nextEma.rollAlpha - ema.rollAlpha) > 0.0001f ||
      fabsf(nextEma.pitchAlpha - ema.pitchAlpha) > 0.0001f ||
      fabsf(nextEma.stopKmh - ema.stopKmh) > 0.0001f)
  {
    ema = nextEma;
    result.emaUpdated = true;
  }

  return result;
}

void keyaLoadWebEmaParams(int eepromAddrYaw,
                          int eepromAddrRoll,
                          int eepromAddrPitch,
                          int eepromAddrStop,
                          KeyaWebEmaParams &ema)
{
  float v = 0.0f;

  EEPROM.get(eepromAddrYaw, v);
  if (!isnan(v) && v >= 0.0f && v <= 1.0f)
    ema.yawAlpha = v;

  EEPROM.get(eepromAddrRoll, v);
  if (!isnan(v) && v >= 0.0f && v <= 1.0f)
    ema.rollAlpha = v;

  EEPROM.get(eepromAddrPitch, v);
  if (!isnan(v) && v >= 0.0f && v <= 1.0f)
    ema.pitchAlpha = v;

  EEPROM.get(eepromAddrStop, v);
  if (!isnan(v) && v >= 0.0f && v <= 20.0f)
    ema.stopKmh = v;
}

void keyaSaveWebEmaParams(int eepromAddrYaw,
                          int eepromAddrRoll,
                          int eepromAddrPitch,
                          int eepromAddrStop,
                          const KeyaWebEmaParams &ema)
{
  EEPROM.put(eepromAddrYaw, ema.yawAlpha);
  EEPROM.put(eepromAddrRoll, ema.rollAlpha);
  EEPROM.put(eepromAddrPitch, ema.pitchAlpha);
  EEPROM.put(eepromAddrStop, ema.stopKmh);
}
