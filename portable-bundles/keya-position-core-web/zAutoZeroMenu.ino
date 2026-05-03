// =============================================================
// SERIAL MENU - Auto-Zero WAS parameter settings (Keya encoder)
// =============================================================
// Usage:
//   - Open the serial monitor (115200 baud)
//   - Type  'z'  then ENTER to display the menu
//   - Type the parameter number, ENTER, then the value, ENTER
//   - Values are saved to EEPROM (address 90)
// =============================================================

#include "KeyaPositionModule.h"

#define EEPROM_ADDR_AZ_PARAMS 90

// Structure AutoZeroParams defined in KeyaPositionModule.h

// Global instance - default values
AutoZeroParams azParams = keyaDefaultAutoZeroParams();

// -----------------------------------------------------------------
// Call in autosteerSetup() after the other EEPROM.get() calls
// -----------------------------------------------------------------
void azMenuSetup()
{
  AutoZeroParams saved;
  EEPROM.get(EEPROM_ADDR_AZ_PARAMS, saved);
  bool hadValidEeprom = keyaIsAutoZeroParamsValid(saved);

  keyaLoadAutoZeroParams(EEPROM_ADDR_AZ_PARAMS, azParams);
  if (hadValidEeprom)
    Serial.println("[AZ-MENU] Parameters loaded from EEPROM.");
  else
    Serial.println("[AZ-MENU] First use - default values saved.");
  azMenuPrint();
}

// -----------------------------------------------------------------
// Display the menu
// -----------------------------------------------------------------
void azMenuPrint()
{
  Serial.println();
  Serial.println("======= AUTO-ZERO WAS MENU =======");
  Serial.print("1. Min speed          : ");
  Serial.print(azParams.speedMin, 1);
  Serial.println(" km/h");
  Serial.print("2. Max yaw rate (BNO) : ");
  Serial.print(azParams.yawRateMax, 2);
  Serial.println(" deg/s  (low=strict)");
  Serial.print("3. Max GPS variation  : ");
  Serial.print(azParams.gpsHdgMax, 2);
  Serial.println(" deg    (low=strict)");
  Serial.print("4. Low speed duration : ");
  Serial.print(azParams.timeSlowMs);
  Serial.println(" ms");
  Serial.print("5. High speed duration: ");
  Serial.print(azParams.timeFastMs);
  Serial.println(" ms");
  Serial.print("6. Low speed threshold: ");
  Serial.print(azParams.speedSlow, 1);
  Serial.println(" km/h");
  Serial.print("7. High speed threshold:");
  Serial.print(azParams.speedFast, 1);
  Serial.println(" km/h");
  Serial.print("8. BNO source         : ");
  Serial.println(azParams.useBno ? "ACTIVE" : "INACTIVE");
  Serial.print("9. GPS source         : ");
  Serial.println(azParams.useGps ? "ACTIVE" : "INACTIVE");
  Serial.print("10. Beta correction   : ");
  Serial.print(azParams.beta, 3);
  Serial.println("  (0.01=slow .. 0.2=fast)");
  Serial.println("11. Reset to defaults");
  Serial.println("12. Quit");
  Serial.println("==================================");
  Serial.println("Type number + ENTER:");
}

// -----------------------------------------------------------------
// Menu loop - call in autosteerLoop() or loop()
// Returns true if the menu is active (blocks other processing)
// -----------------------------------------------------------------
static bool azMenuActive = false;
static uint8_t azMenuStep = 0; // 0=waiting for choice, 1=waiting for value
static uint8_t azMenuChoice = 0;

bool azMenuLoop()
{
  // Detect 'z' key press outside menu
  if (!azMenuActive)
  {
    if (Serial.available())
    {
      String input = Serial.readStringUntil('\n');
      input.trim();

      // EMA BNO filter commands (EY / ER)
      if (handleEmaSerialCommand(input))
        return false;

      // Activate auto-zero menu
      if (input == "z" || input == "Z")
      {
        azMenuActive = true;
        azMenuStep = 0;
        azMenuPrint();
      }
    }
    return false;
  }

  // Menu active
  if (!Serial.available())
    return true;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0)
    return true;

  if (azMenuStep == 0)
  {
    // Read choice
    azMenuChoice = input.toInt();

    if (azMenuChoice == 11)
    {
      // Reset to defaults
      azParams = keyaDefaultAutoZeroParams();
      keyaSaveAutoZeroParams(EEPROM_ADDR_AZ_PARAMS, azParams);
      Serial.println("[AZ-MENU] Default values restored and saved.");
      azMenuPrint();
      return true;
    }

    if (azMenuChoice == 12)
    {
      Serial.println("[AZ-MENU] Menu closed. Type 'z' to reopen.");
      azMenuActive = false;
      azMenuStep = 0;
      return false;
    }

    if (azMenuChoice >= 1 && azMenuChoice <= 10)
    {
      if (azMenuChoice == 8 || azMenuChoice == 9)
      {
        Serial.print("New value (0=inactive, 1=active) for parameter ");
      }
      else
      {
        Serial.print("New value for parameter ");
      }
      Serial.print(azMenuChoice);
      Serial.println(":");
      azMenuStep = 1;
    }
    else
    {
      Serial.println("Invalid choice.");
      azMenuPrint();
    }
  }
  else if (azMenuStep == 1)
  {
    // Read value
    float val = input.toFloat();

    switch (azMenuChoice)
    {
    case 1:
      azParams.speedMin = val;
      break;
    case 2:
      azParams.yawRateMax = val;
      break;
    case 3:
      azParams.gpsHdgMax = val;
      break;
    case 4:
      azParams.timeSlowMs = (uint32_t)val;
      break;
    case 5:
      azParams.timeFastMs = (uint32_t)val;
      break;
    case 6:
      azParams.speedSlow = val;
      break;
    case 7:
      azParams.speedFast = val;
      break;
    case 8:
      azParams.useBno = (val >= 1.0f) ? 1 : 0;
      break;
    case 9:
      azParams.useGps = (val >= 1.0f) ? 1 : 0;
      break;
    case 10:
      if (val >= 0.001f && val <= 1.0f)
        azParams.beta = val;
      else
        Serial.println("Beta out of range (0.001 - 1.0), ignored.");
      break;
    }

    keyaSaveAutoZeroParams(EEPROM_ADDR_AZ_PARAMS, azParams);
    Serial.println("[AZ-MENU] Saved OK.");
    azMenuStep = 0;
    azMenuPrint();
  }

  return true;
}
