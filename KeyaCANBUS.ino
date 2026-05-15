#define lowByte(w) ((uint8_t)((w) & 0xFF))
#define highByte(w) ((uint8_t)((w) >> 8))

#include "KeyaPositionModule.h"

uint8_t KeyaSteerPGN[] = {0x23, 0x00, 0x20, 0x01, 0, 0, 0, 0}; // last 4 bytes change ofc
uint8_t KeyaHeartbeat[] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
};

// templates for matching responses of interest
uint8_t keyaCurrentResponse[] = {0x60, 0x12, 0x21, 0x01};

uint64_t KeyaPGN = 0x06000001;

const bool debugKeya = false;

void keyaSend(uint8_t data[], size_t length)
{
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  memcpy(KeyaBusSendData.buf, data, length);
  Keya_Bus.write(KeyaBusSendData);
}

void CAN_Setup()
{
  canbus1.begin();
  canbus1.setBaudRate(250000);
  init_canbus1_filters();
	Serial.println("Joystick CANBUS setup complete");
  Keya_Bus.begin();
  Keya_Bus.setBaudRate(250000);
  delay(1000);
}

bool isPatternMatch(const CAN_message_t &message, const uint8_t *pattern, size_t patternSize)
{
  return memcmp(message.buf, pattern, patternSize) == 0;
}

void disableKeyaSteer()
{
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x23;
  KeyaBusSendData.buf[1] = 0x0c;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x01;
  KeyaBusSendData.buf[4] = 0;
  KeyaBusSendData.buf[5] = 0;
  KeyaBusSendData.buf[6] = 0;
  KeyaBusSendData.buf[7] = 0;
  Keya_Bus.write(KeyaBusSendData);
}

void disableKeyaSteerTEST()
{
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x03;
  KeyaBusSendData.buf[1] = 0x0d;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x11;
  KeyaBusSendData.buf[4] = 0;
  KeyaBusSendData.buf[5] = 0;
  KeyaBusSendData.buf[6] = 0;
  KeyaBusSendData.buf[7] = 0;
  Keya_Bus.write(KeyaBusSendData);
}

void enableKeyaSteer()
{
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x23;
  KeyaBusSendData.buf[1] = 0x0d;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x01;
  KeyaBusSendData.buf[4] = 0;
  KeyaBusSendData.buf[5] = 0;
  KeyaBusSendData.buf[6] = 0;
  KeyaBusSendData.buf[7] = 0;
  Keya_Bus.write(KeyaBusSendData);
  if (debugKeya)
    Serial.println("Enabled Keya motor");
}

void SteerKeya(int steerSpeed)
{
  int actualSpeed = map(steerSpeed, -255, 255, -995, 998);
  if (pwmDrive == 0)
  {
    disableKeyaSteer();
  }
  if (debugKeya)
    Serial.println("told to steer, with " + String(steerSpeed) + " so....");
  if (debugKeya)
    Serial.println("I converted that to speed " + String(actualSpeed));

  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x23;
  KeyaBusSendData.buf[1] = 0x00;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x01;
  if (steerSpeed < 0)
  {
    KeyaBusSendData.buf[4] = highByte(actualSpeed); // TODO take PWM in instead for speed (this is -1000)
    KeyaBusSendData.buf[5] = lowByte(actualSpeed);
    KeyaBusSendData.buf[6] = 0xff;
    KeyaBusSendData.buf[7] = 0xff;
    if (debugKeya)
      Serial.println("pwmDrive < zero - clockwise - steerSpeed " + String(steerSpeed));
  }
  else
  {
    KeyaBusSendData.buf[4] = highByte(actualSpeed);
    KeyaBusSendData.buf[5] = lowByte(actualSpeed);
    KeyaBusSendData.buf[6] = 0x00;
    KeyaBusSendData.buf[7] = 0x00;
    if (debugKeya)
      Serial.println("pwmDrive > zero - anticlock-clockwise - steerSpeed " + String(steerSpeed));
  }
  Keya_Bus.write(KeyaBusSendData);
  enableKeyaSteer();
}

// ---------------------------------------------------------------------------
// Keya encoder – cumulative position from the heartbeat (bytes 0-1)
// Unit: 65535 ticks = 1 motor revolution = 360° motor
// The hardware counter is uint16 (0-65535) and can overflow in both directions.
// Deltas are accumulated in a signed int32 to obtain an absolute position.
// ---------------------------------------------------------------------------
#define KEYA_ENCODER_INVERT 1 // 0 = normal direction | 1 = reverse direction

int32_t keyaEncoderRaw = 0;
uint16_t keyaEncPrev = 0;
bool keyaEncInitDone = false;

void keyaUpdateEncoder(uint16_t rawTick)
{
  KeyaEncoderRuntime runtime = {
      .encoderRaw = keyaEncoderRaw,
      .prevTick = keyaEncPrev,
      .initDone = keyaEncInitDone};

  keyaUpdateEncoderFromHeartbeat(runtime,
                                 rawTick,
                                 KEYA_ENCODER_INVERT != 0);

  keyaEncoderRaw = runtime.encoderRaw;
  keyaEncPrev = runtime.prevTick;
  keyaEncInitDone = runtime.initDone;
}

void KeyaBus_Receive()
{
  CAN_message_t KeyaBusReceiveData;
  if (Keya_Bus.read(KeyaBusReceiveData))
  {
    if (KeyaBusReceiveData.id == 0x07000001)
    {
      lastKeyaHeatbeat = 0;

      if (!keyaDetected)
      {
        keyaDetected = true;
        Serial.println("Keya heartbeat detected! Enabling Keya CANBUS");
      }

      // --- Cumulative encoder (bytes 0-1, high byte first per manual) ---
      uint16_t encTick = ((uint16_t)KeyaBusReceiveData.buf[0] << 8) | (uint16_t)KeyaBusReceiveData.buf[1];
      keyaUpdateEncoder(encTick);

      // --- Motor current (bytes 4-5, unchanged) ---
      if (KeyaBusReceiveData.buf[4] == 0xFF)
      {
        KeyaCurrentSensorReading = (0.95 * KeyaCurrentSensorReading) + (0.05 * (256 - KeyaBusReceiveData.buf[5]) * 20);
      }
      else
      {
        KeyaCurrentSensorReading = (0.95 * KeyaCurrentSensorReading) + (0.05 * KeyaBusReceiveData.buf[5] * 20);
      }
    }
  }
}
