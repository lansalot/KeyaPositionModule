#ifndef KEYA_POSITION_MODULE_H
#define KEYA_POSITION_MODULE_H

#include <stdint.h>

// Canonical Auto-Zero parameter model for Danfoss/Keya mode.
struct AutoZeroParams
{
  float speedMin;
  float yawRateMax;
  float gpsHdgMax;
  uint32_t timeSlowMs;
  uint32_t timeFastMs;
  float speedSlow;
  float speedFast;
  uint8_t useBno;
  uint8_t useGps;
  float beta;
  uint16_t ident;
};

struct KeyaAutoZeroRuntime
{
  float corrAccum;
  uint32_t stableStartMs;
  uint32_t cooldownStampMs;

  float lastYawDeg;
  uint32_t lastYawTimeMs;
  bool yawInitDone;

  float lastGpsHeadingDeg;
  bool gpsInitDone;

  int64_t accumTicks;
  uint32_t accumCount;
};

struct KeyaEncoderRuntime
{
  int32_t encoderRaw;
  uint16_t prevTick;
  bool initDone;
};

struct KeyaAutoZeroInput
{
  uint32_t nowMs;
  float gpsSpeedKmh;
  float yawDeg;
  float emaGpsHeadingX10Deg;
  float steerAngleActualDeg;
  bool invertWas;
  bool guidanceActive;
};

struct KeyaAutoZeroConfig
{
  float nearZeroDeg;
  float nearZeroFactor;
  uint32_t cooldownMs;
  float minStableMs;
  float maxStableMs;
};

AutoZeroParams keyaDefaultAutoZeroParams();
bool keyaIsAutoZeroParamsValid(const AutoZeroParams &params);
void keyaLoadAutoZeroParams(int eepromAddr, AutoZeroParams &params);
void keyaSaveAutoZeroParams(int eepromAddr, const AutoZeroParams &params);

float keyaSanitizeTicksPerDeg(float ticksPerDeg, float fallbackTicksPerDeg);
void keyaLoadTicksPerDeg(int eepromAddr, float &ticksPerDeg, float fallbackTicksPerDeg);
void keyaSaveTicksPerDeg(int eepromAddr, float ticksPerDeg, float fallbackTicksPerDeg);

void keyaInitEncoderRuntime(KeyaEncoderRuntime &encoderRuntime);
void keyaUpdateEncoderFromHeartbeat(KeyaEncoderRuntime &encoderRuntime, uint16_t rawTick, bool invertDirection);

float keyaComputeAngleDeg(int32_t encoderRaw, int32_t zeroTicks, float ticksPerDeg, bool invertWas);
int32_t keyaComputeDeltaTicks(int32_t encoderRaw, int32_t zeroTicks);

void keyaApplyAogSteerSensorCounts(uint8_t steerSensorCounts, float defaultTicksPerDeg, float &ticksPerDeg);
void keyaForceZeroAtCurrentEncoder(int32_t encoderRaw, int32_t &zeroTicks, bool &zeroDone, KeyaAutoZeroRuntime &runtime);

void keyaInitAutoZeroRuntime(KeyaAutoZeroRuntime &runtime);
KeyaAutoZeroConfig keyaDefaultAutoZeroConfig();

bool keyaUpdateAutoZero(const KeyaAutoZeroInput &input,
                        const AutoZeroParams &params,
                        const KeyaAutoZeroConfig &config,
                        int32_t encoderRaw,
                        float ticksPerDeg,
                        int32_t &zeroTicks,
                        bool &zeroDone,
                        KeyaAutoZeroRuntime &runtime);

#endif