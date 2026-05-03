#include "KeyaPositionModule.h"
#include <Arduino.h>
#include <EEPROM.h>

namespace
{
constexpr uint16_t KEYA_AZ_IDENT = 0xA202;
constexpr float KEYA_TICKS_PER_DEG_MIN = 1.0f;
constexpr float KEYA_TICKS_PER_DEG_MAX = 500.0f;

static uint32_t keyaComputeStableDurationMs(float gpsSpeedKmh, const AutoZeroParams &params)
{
  float stableMs = 0.0f;

  if (gpsSpeedKmh <= params.speedSlow)
    stableMs = (float)params.timeSlowMs;
  else if (gpsSpeedKmh >= params.speedFast)
    stableMs = (float)params.timeFastMs;
  else
  {
    float denom = params.speedFast - params.speedSlow;
    if (fabsf(denom) < 0.001f)
      denom = 0.001f;
    float t = (gpsSpeedKmh - params.speedSlow) / denom;
    stableMs = (float)params.timeSlowMs + t * ((float)params.timeFastMs - (float)params.timeSlowMs);
  }

  return (uint32_t)constrain(stableMs, 200.0f, 5000.0f);
}

static float keyaWrapAbsDeltaDeg(float nowDeg, float prevDeg)
{
  float d = nowDeg - prevDeg;
  if (d > 180.0f)
    d -= 360.0f;
  if (d < -180.0f)
    d += 360.0f;
  return fabsf(d);
}

} // namespace

AutoZeroParams keyaDefaultAutoZeroParams()
{
  AutoZeroParams params;
  params.speedMin = 2.0f;
  params.yawRateMax = 0.5f;
  params.gpsHdgMax = 0.5f;
  params.timeSlowMs = 500;
  params.timeFastMs = 200;
  params.speedSlow = 3.0f;
  params.speedFast = 12.0f;
  params.useBno = 1;
  params.useGps = 1;
  params.beta = 0.05f;
  params.ident = KEYA_AZ_IDENT;
  return params;
}

bool keyaIsAutoZeroParamsValid(const AutoZeroParams &params)
{
  if (params.ident != KEYA_AZ_IDENT)
    return false;

  if (!isfinite(params.speedMin) || params.speedMin < 0.0f || params.speedMin > 25.0f)
    return false;
  if (!isfinite(params.yawRateMax) || params.yawRateMax < 0.0f || params.yawRateMax > 10.0f)
    return false;
  if (!isfinite(params.gpsHdgMax) || params.gpsHdgMax < 0.0f || params.gpsHdgMax > 10.0f)
    return false;
  if (params.timeSlowMs < 100 || params.timeSlowMs > 10000)
    return false;
  if (params.timeFastMs < 100 || params.timeFastMs > 10000)
    return false;
  if (!isfinite(params.speedSlow) || !isfinite(params.speedFast))
    return false;
  if (params.speedSlow < 0.1f || params.speedSlow > 50.0f)
    return false;
  if (params.speedFast < 0.1f || params.speedFast > 50.0f)
    return false;
  if (params.useBno > 1 || params.useGps > 1)
    return false;
  if (!isfinite(params.beta) || params.beta < 0.001f || params.beta > 1.0f)
    return false;

  return true;
}

void keyaLoadAutoZeroParams(int eepromAddr, AutoZeroParams &params)
{
  AutoZeroParams saved;
  EEPROM.get(eepromAddr, saved);
  if (keyaIsAutoZeroParamsValid(saved))
  {
    params = saved;
    return;
  }

  params = keyaDefaultAutoZeroParams();
  EEPROM.put(eepromAddr, params);
}

void keyaSaveAutoZeroParams(int eepromAddr, const AutoZeroParams &params)
{
  AutoZeroParams toStore = params;
  toStore.ident = KEYA_AZ_IDENT;
  EEPROM.put(eepromAddr, toStore);
}

float keyaSanitizeTicksPerDeg(float ticksPerDeg, float fallbackTicksPerDeg)
{
  if (isfinite(ticksPerDeg) && ticksPerDeg > KEYA_TICKS_PER_DEG_MIN && ticksPerDeg < KEYA_TICKS_PER_DEG_MAX)
    return ticksPerDeg;
  return fallbackTicksPerDeg;
}

void keyaLoadTicksPerDeg(int eepromAddr, float &ticksPerDeg, float fallbackTicksPerDeg)
{
  float saved = fallbackTicksPerDeg;
  EEPROM.get(eepromAddr, saved);
  ticksPerDeg = keyaSanitizeTicksPerDeg(saved, fallbackTicksPerDeg);
}

void keyaSaveTicksPerDeg(int eepromAddr, float ticksPerDeg, float fallbackTicksPerDeg)
{
  float sanitized = keyaSanitizeTicksPerDeg(ticksPerDeg, fallbackTicksPerDeg);
  EEPROM.put(eepromAddr, sanitized);
}

void keyaInitEncoderRuntime(KeyaEncoderRuntime &encoderRuntime)
{
  encoderRuntime.encoderRaw = 0;
  encoderRuntime.prevTick = 0;
  encoderRuntime.initDone = false;
}

void keyaUpdateEncoderFromHeartbeat(KeyaEncoderRuntime &encoderRuntime, uint16_t rawTick, bool invertDirection)
{
  if (!encoderRuntime.initDone)
  {
    encoderRuntime.prevTick = rawTick;
    encoderRuntime.initDone = true;
    return;
  }

  int16_t delta = (int16_t)(rawTick - encoderRuntime.prevTick);
  if (invertDirection)
    delta = -delta;

  encoderRuntime.encoderRaw += delta;
  encoderRuntime.prevTick = rawTick;
}

float keyaComputeAngleDeg(int32_t encoderRaw, int32_t zeroTicks, float ticksPerDeg, bool invertWas)
{
  float sanitizedTicks = keyaSanitizeTicksPerDeg(ticksPerDeg, 24.0f);
  int32_t deltaTicks = encoderRaw - zeroTicks;
  float angleDeg = (float)deltaTicks / sanitizedTicks;
  if (invertWas)
    angleDeg = -angleDeg;
  return angleDeg;
}

int32_t keyaComputeDeltaTicks(int32_t encoderRaw, int32_t zeroTicks)
{
  return encoderRaw - zeroTicks;
}

void keyaApplyAogSteerSensorCounts(uint8_t steerSensorCounts, float defaultTicksPerDeg, float &ticksPerDeg)
{
  ticksPerDeg = defaultTicksPerDeg * (0.5f + (float)steerSensorCounts / 255.0f);
}

void keyaForceZeroAtCurrentEncoder(int32_t encoderRaw, int32_t &zeroTicks, bool &zeroDone, KeyaAutoZeroRuntime &runtime)
{
  zeroTicks = encoderRaw;
  zeroDone = true;
  runtime.corrAccum = 0.0f;
  runtime.stableStartMs = 0;
  runtime.accumTicks = 0;
  runtime.accumCount = 0;
}

void keyaInitAutoZeroRuntime(KeyaAutoZeroRuntime &runtime)
{
  runtime.corrAccum = 0.0f;
  runtime.stableStartMs = 0;
  runtime.cooldownStampMs = 0;
  runtime.lastYawDeg = 0.0f;
  runtime.lastYawTimeMs = 0;
  runtime.yawInitDone = false;
  runtime.lastGpsHeadingDeg = 0.0f;
  runtime.gpsInitDone = false;
  runtime.accumTicks = 0;
  runtime.accumCount = 0;
}

KeyaAutoZeroConfig keyaDefaultAutoZeroConfig()
{
  KeyaAutoZeroConfig cfg;
  cfg.nearZeroDeg = 2.0f;
  cfg.nearZeroFactor = 0.3f;
  cfg.cooldownMs = 2000;
  cfg.minStableMs = 200.0f;
  cfg.maxStableMs = 5000.0f;
  return cfg;
}

bool keyaUpdateAutoZero(const KeyaAutoZeroInput &input,
                        const AutoZeroParams &params,
                        const KeyaAutoZeroConfig &config,
                        int32_t encoderRaw,
                        float ticksPerDeg,
                        int32_t &zeroTicks,
                        bool &zeroDone,
                        KeyaAutoZeroRuntime &runtime)
{
  float yawRate = 0.0f;
  if (!runtime.yawInitDone)
  {
    runtime.lastYawDeg = input.yawDeg;
    runtime.lastYawTimeMs = input.nowMs;
    runtime.yawInitDone = true;
  }
  else
  {
    float dt = (input.nowMs - runtime.lastYawTimeMs) / 1000.0f;
    if (dt < 0.001f)
      dt = 0.001f;
    yawRate = keyaWrapAbsDeltaDeg(input.yawDeg, runtime.lastYawDeg) / dt;
    runtime.lastYawDeg = input.yawDeg;
    runtime.lastYawTimeMs = input.nowMs;
  }

  float gpsHeadingDeg = input.emaGpsHeadingX10Deg / 10.0f;
  float gpsHdgRate = 0.0f;
  if (!runtime.gpsInitDone)
  {
    runtime.lastGpsHeadingDeg = gpsHeadingDeg;
    runtime.gpsInitDone = true;
  }
  else
  {
    gpsHdgRate = keyaWrapAbsDeltaDeg(gpsHeadingDeg, runtime.lastGpsHeadingDeg);
    runtime.lastGpsHeadingDeg = gpsHeadingDeg;
  }

  float adaptFactor = 1.0f;
  if (input.guidanceActive)
  {
    float absAngle = fabsf(input.steerAngleActualDeg);
    if (absAngle < config.nearZeroDeg)
    {
      float ratio = absAngle / max(0.001f, config.nearZeroDeg);
      adaptFactor = config.nearZeroFactor + ratio * (1.0f - config.nearZeroFactor);
    }
  }

  float yawRateMax = params.yawRateMax * adaptFactor;
  float gpsHdgMax = params.gpsHdgMax * adaptFactor;

  bool speedOk = (input.gpsSpeedKmh > params.speedMin);
  bool straightOk = (!params.useBno) || (yawRate < yawRateMax);
  bool gpsOk = (!params.useGps) || (gpsHdgRate < gpsHdgMax);
  bool cooldownOk = (input.nowMs - runtime.cooldownStampMs > config.cooldownMs);

  if (!(speedOk && straightOk && gpsOk && cooldownOk))
  {
    runtime.stableStartMs = 0;
    runtime.accumTicks = 0;
    runtime.accumCount = 0;
    return false;
  }

  if (runtime.stableStartMs == 0)
  {
    runtime.stableStartMs = input.nowMs;
    runtime.accumTicks = 0;
    runtime.accumCount = 0;
  }

  runtime.accumTicks += (int64_t)encoderRaw;
  runtime.accumCount++;

  uint32_t requiredStableMs = keyaComputeStableDurationMs(input.gpsSpeedKmh, params);
  requiredStableMs = (uint32_t)constrain((float)requiredStableMs, config.minStableMs, config.maxStableMs);

  if ((input.nowMs - runtime.stableStartMs) <= requiredStableMs || runtime.accumCount == 0)
    return false;

  int32_t meanTicks = (int32_t)(runtime.accumTicks / (int64_t)runtime.accumCount);
  float sanitizedTicks = keyaSanitizeTicksPerDeg(ticksPerDeg, 24.0f);

  if (!zeroDone)
  {
    zeroTicks = meanTicks;
    zeroDone = true;
    runtime.corrAccum = 0.0f;
  }
  else if (!input.guidanceActive)
  {
    zeroTicks = meanTicks;
    runtime.corrAccum = 0.0f;
  }
  else
  {
    float corrSign = input.invertWas ? -1.0f : 1.0f;
    runtime.corrAccum += corrSign * params.beta * input.steerAngleActualDeg * sanitizedTicks;
    int32_t corrInt = (int32_t)runtime.corrAccum;
    if (corrInt != 0)
    {
      zeroTicks += corrInt;
      runtime.corrAccum -= (float)corrInt;
    }
  }

  runtime.accumTicks = 0;
  runtime.accumCount = 0;
  runtime.stableStartMs = 0;
  runtime.cooldownStampMs = input.nowMs;
  return true;
}
