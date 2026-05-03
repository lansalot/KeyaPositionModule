// SectionControl.ino
// Section control for AutoSteer Teensy 4.1 FlorianT
// Supported modes: GPIO Teensy | I2C XL9535 | CAN Bus
//
// ============================================================================
// RESERVED PINS (DO NOT USE):
//   0,1,3,4,9,5,24,6,7,8,14,15,16,17,18,19,22,23,30,31,A0,A1,A2
//
// AVAILABLE PINS FOR SECTION CONTROL:
//   2, 10, 11, 12, 13, 20, 21, 25, 26, 27, 28, 29, 32, 33
//   34, 35, 36, 37, 38, 39, 40, 41 (A16-A23)
// ============================================================================

// ============================================================================
//   CONFIG - EDIT HERE ONLY
// ============================================================================

// --- Relay output mode ---
// 0 = GPIO Teensy
// 1 = I2C XL9535
// 2 = CAN Bus
#define SC_OUTPUT_MODE 2

// --- Auto/Manual mode switch source ---
//   1-41   = GPIO Teensy (direct pin number): HIGH = AUTO, LOW = MANUAL (INPUT_PULLUP)
//   101-116 = CAN input (101=IN1 ... 116=IN16): active (1) = MANUAL, inactive = AUTO
//   0      = Disabled (always AUTO)
#define SC_AUTO_MANUAL_INPUT 108 // ex: 108 = CAN IN8, 12 = GPIO pin 12

// --- Relay behaviour in MANUAL mode ---
// 0 = Send info to AOG only (AOG decides relays)
//     → Manual buttons inform AOG via PGN 234, but physical relays stay OFF
// 1 = Activate physical relays AND send info to AOG
//     → Manual buttons drive relays directly + inform AOG
#define SC_MANUAL_RELAY_MODE 0

// ============================================================================
// MASTER VALVE (MV) CONFIG
// ============================================================================
// Enable master valve management
// 0 = Disabled
// 1 = Enabled
#define VG_ENABLE 1

// Master valve button — same convention as manualInputMap:
//   1-41    = GPIO Teensy (INPUT_PULLUP, LOW=ON, HIGH=OFF)
//   101-116 = CAN IN (active=ON, inactive=OFF)
//   0       = Disabled
#define VG_PIN_BUTTON 11

// GPIO pin for the master valve relay (OUTPUT)
// Set to 0 to disable relay output
#define VG_PIN_RELAY 12

// Delay before automatic master valve close when all sections are OFF (in ms)
// Default 5000ms = 5 seconds
#define VG_CLOSE_DELAY_MS 5000

// ============================================================================
// GPIO RELAY OUTPUT CONFIG (GPIO mode only)
// ============================================================================
uint8_t teensy_pins[] = {33};
uint8_t NUM_CONFIGURABLE_PINS = sizeof(teensy_pins) / sizeof(teensy_pins[0]);

// 0        = unassigned
// 1-41     = GPIO Teensy (direct pin number, check availability)
// 101-108  = CAN input (101=IN1, 102=IN2 ... 108=IN8)
struct ManualInput
{
    uint8_t source;
} manualInputMap[16] = {
    {101}, // Section 1  → CAN IN1
    {102}, // Section 2  → CAN IN2
    {103}, // Section 3  → GPIO pin 2
    {104}, // Section 4  → GPIO pin 10
    {105}, // Section 5  → CAN IN3
    {106}, // Section 6  → unassigned
    {107}, // Section 7  → unassigned
    {0},   // Section 8  → unassigned
    {0},   // Section 9  → unassigned
    {0},   // Section 10 → unassigned
    {0},   // Section 11 → unassigned
    {0},   // Section 12 → unassigned
    {0},   // Section 13 → unassigned
    {0},   // Section 14 → unassigned
    {0},   // Section 15 → unassigned
    {0},   // Section 16 → unassigned
};
// ============================================================================
//  END CONFIG
// ============================================================================

// --- I2C config (if SC_OUTPUT_MODE == 1) ---
// XL9535 address (A2=A1=A0=0 → 0x20)
#define XL9535_ADDRESS 0x20

// --- CAN config (if SC_OUTPUT_MODE == 2) ---
#define SC_CAN_MODULE_ADDR 0x01 // CAN relay module address
#define SC_CAN_INPUT_POLL_MS 50 // CAN input polling interval (ms)

// Calculated CAN IDs (do not modify unless using a different protocol)
#define SC_CAN_ID_WRITE_RELAYS ((0x01 << 8) | SC_CAN_MODULE_ADDR) // ex: 0x101
#define SC_CAN_ID_READ_INPUTS ((0x03 << 8) | SC_CAN_MODULE_ADDR)  // ex: 0x301

#define SC_GPIO 0
#define SC_I2C 1
#define SC_CAN 2

// ============================================================================
// EXTERNAL VARIABLES (defined in the main .ino)
// ============================================================================
// Each host sketch must declare ONE macro at the very top, before includes:
//
//   Tony      →  #define AOG_SKETCH_TONY
//   Autosteer →  #define AOG_SKETCH_AUTOSTEER
//   Future    →  #define AOG_SKETCH_<NAME>  (add a #elif block below)
//
// The rest of the code always uses: Udp / ipDestination / AOGPort
// ============================================================================

#if defined(AOG_SKETCH_TONY)
// --- Sketch Tony ---
extern EthernetUDP Udp;
extern uint8_t ipDestination[];
extern unsigned int AOGPort;

#elif defined(AOG_SKETCH_KEYA)
// --- Sketch Autosteer ---
extern EthernetUDP Eth_udpAutoSteer;
extern IPAddress Eth_ipDestination;
extern unsigned int portDestination;
// Transparent aliases → rest of code always uses the same names
#define Udp Eth_udpAutoSteer
#define ipDestination Eth_ipDestination
#define AOGPort portDestination

// #elif defined(AOG_SKETCH_MY_THING)
//   extern ...

#else
#error "SectionControl: no host sketch defined. Add #define AOG_SKETCH_TONY or AOG_SKETCH_KEYA before the includes of the main sketch."
#endif

extern uint8_t relay;
extern uint8_t relayHi;
extern uint8_t tram;
extern uint8_t uTurn;
extern uint8_t hydLift;
extern float gpsSpeed;
extern Config aogConfig;

// ============================================================================
// PIN/RELAY MAPPING ARRAYS
// ============================================================================

// AOG → physical function mapping (received from AgOpenGPS, saved EEPROM addr 120)
// index = configurable output position
// value: 0=none | 1-16=section | 17=HydUp | 18=HydDown | 19=TramR | 20=TramL | 21=GeoStop
uint8_t pin[24] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Calculated logical states (independent of output mode)
uint8_t relayState[23] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

bool isRaise = false, isLower = false;
uint8_t raiseTimer = 0, lowerTimer = 0, lastTrigger = 0;

// ============================================================================
// VARIABLES I2C XL9535
// ============================================================================
#define XL9535_OUTPUT_PORT0 0x02
#define XL9535_OUTPUT_PORT1 0x03
#define XL9535_CONFIG_PORT0 0x06
#define XL9535_CONFIG_PORT1 0x07

uint16_t i2cRelayStates = 0;

// ============================================================================
// CAN VARIABLES
// ============================================================================
volatile uint8_t canInputStates = 0x00;  // IN1-IN8 bits read from module
volatile uint8_t canInputStates2 = 0x00; // IN9-IN16
uint32_t lastCANInputPoll = 0;

// ============================================================================
// PGN
// ============================================================================
uint8_t PGN_237[] = {0x80, 0x81, 0x7f, 237, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC};
int8_t PGN_237_Size = sizeof(PGN_237) - 1;

uint8_t PGN_234[] = {0x80, 0x81, 0x7B, 234, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC};
int8_t PGN_234_Size = sizeof(PGN_234) - 1;

// ============================================================================
// TIMERS
// ============================================================================
uint32_t lastHydraulicTime = 0;
const uint16_t HYDRAULIC_LOOP_TIME = 200; // 200ms = 5Hz

// ============================================================================
// AUTO/MANUAL MODE VARIABLES
// ============================================================================
bool autoModeIsOn = true;
uint8_t onLo = 0, offLo = 0;
uint8_t onHi = 0, offHi = 0;
uint8_t mainByte = 1; // 1=Auto, 2=Manual

// ============================================================================
// MASTER VALVE (MV) VARIABLES
// ============================================================================
bool vgIsOpen = false;             // Current MV state (true=open)
bool vgButtonLastState = true;     // Last button state read (INPUT_PULLUP → true=released)
bool vgButtonPending = false;      // Falling edge detected, waiting for debounce
uint32_t vgButtonPressTime = 0;    // Timestamp of falling edge
uint32_t vgAllOffSince = 0;        // Timestamp when all sections went OFF
bool vgAllOffTimerActive = false;  // Auto-close timer running
const uint8_t VG_DEBOUNCE_MS = 50; // Button debounce (ms)
uint8_t vgForcedOnLo = 0;          // Sections 1-8  active at MV ON toggle
uint8_t vgForcedOnHi = 0;          // Sections 9-16 active at MV ON toggle
bool vgActive = false;             // true=MV activated by button, sections follow vgForcedOn

//  FONCTIONS I2C XL9535
void initXL9535()
{
#if (SC_OUTPUT_MODE == SC_I2C)
    Wire2.begin();
    delay(50);

    Wire2.beginTransmission(XL9535_ADDRESS);
    Wire2.write(XL9535_CONFIG_PORT0);
    Wire2.write(0x00); // Port 0: outputs
    Wire2.write(0x00); // Port 1: outputs
    uint8_t error = Wire2.endTransmission();

    if (error == 0)
    {
        Serial.println("  ✓ XL9535 initialized");
        i2cRelayStates = aogConfig.isRelayActiveHigh ? 0x0000 : 0xFFFF;
        updateAllI2CRelays();
    }
    else
    {
        Serial.print("  ✗ XL9535 error: ");
        Serial.println(error);
    }
#endif
}

void setI2CRelay(uint8_t relayNumber, bool state)
{
#if (SC_OUTPUT_MODE == SC_I2C)
    if (relayNumber >= 16)
        return;
    bool physicalState = aogConfig.isRelayActiveHigh ? state : !state;
    if (physicalState)
        i2cRelayStates |= (1 << relayNumber);
    else
        i2cRelayStates &= ~(1 << relayNumber);
#endif
}

void updateAllI2CRelays()
{
#if (SC_OUTPUT_MODE == SC_I2C)
    Wire2.beginTransmission(XL9535_ADDRESS);
    Wire2.write(XL9535_OUTPUT_PORT0);
    Wire2.write(i2cRelayStates & 0xFF);
    Wire2.write((i2cRelayStates >> 8) & 0xFF);
    Wire2.endTransmission();
#endif
}

//  FONCTIONS CAN BUS
void initSectionCAN()
{
#if (SC_OUTPUT_MODE == SC_CAN)

    Sc_Bus.begin();
    Sc_Bus.setBaudRate(250000);

    uint8_t initVal = aogConfig.isRelayActiveHigh ? 0x00 : 0xFF;
    updateAllCANRelays(initVal, initVal);

    Serial.print("  ✓ Sc_Bus CAN ready - ID Write: 0x");
    Serial.print(SC_CAN_ID_WRITE_RELAYS, HEX);
    Serial.print("  ID Read: 0x");
    Serial.println(SC_CAN_ID_READ_INPUTS, HEX);
#endif
}

// Send state of 16 relays: relaysLo=1-8, relaysHi=9-16
void updateAllCANRelays(uint8_t relaysLo, uint8_t relaysHi)
{
#if (SC_OUTPUT_MODE == SC_CAN)
    CAN_message_t msg;
    msg.id = SC_CAN_ID_WRITE_RELAYS; // 0x101
    msg.len = 8;
    msg.buf[0] = relaysLo; // Relays 1-8
    msg.buf[1] = relaysHi; // Relays 9-16
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x00;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    Sc_Bus.write(msg);
#endif
}

void readCANInputs()
{
#if (SC_OUTPUT_MODE == SC_CAN)
    // Send input read request
    CAN_message_t req;
    req.id = SC_CAN_ID_READ_INPUTS; // 0x301
    req.len = 8;
    for (uint8_t i = 0; i < 8; i++)
        req.buf[i] = 0x00;
    Sc_Bus.write(req);
#endif
}

// Passive CAN read: call continuously in loop (non-blocking)
void pollCANMessages()
{
#if (SC_OUTPUT_MODE == SC_CAN)
    CAN_message_t rx;
    while (Sc_Bus.read(rx))
    {
        if (rx.id == SC_CAN_ID_READ_INPUTS)
        {
            canInputStates = rx.buf[0];  // IN1-IN8
            canInputStates2 = rx.buf[1]; // IN9-IN16
        }
        // Other messages (relay ACK etc.) are simply ignored
    }
#endif
}

// Returns true=AUTO / false=MANUAL based on SC_AUTO_MANUAL_INPUT
// Convention: 1-41 = GPIO Teensy, 101-116 = CAN IN1-IN16, 0 = always AUTO
bool getAutoManual()
{
#if (SC_AUTO_MANUAL_INPUT >= 1) && (SC_AUTO_MANUAL_INPUT <= 41)
    return digitalRead(SC_AUTO_MANUAL_INPUT); // HIGH=AUTO, LOW=MANUAL (INPUT_PULLUP)
#elif (SC_AUTO_MANUAL_INPUT >= 101) && (SC_AUTO_MANUAL_INPUT <= 108)
    return !bitRead(canInputStates, SC_AUTO_MANUAL_INPUT - 101); // active=MANUAL
#elif (SC_AUTO_MANUAL_INPUT >= 109) && (SC_AUTO_MANUAL_INPUT <= 116)
    return !bitRead(canInputStates2, SC_AUTO_MANUAL_INPUT - 109);
#else
    return true; // SC_AUTO_MANUAL_INPUT == 0: always AUTO
#endif
}

//  SETUP
void sectionControlSetup()
{
    EEPROM.get(120, pin);

    Serial.println("\r\n=== Section Control ===");

    // ---- Output mode ----
#if (SC_OUTPUT_MODE == SC_CAN)
    Serial.println("  Output mode: CAN Bus (Sc_Bus)");
    initSectionCAN();

    Serial.println("  CAN input → manual section mapping:");
    for (uint8_t i = 0; i < 8; i++)
    {
        Serial.println("  Manual input mapping per section:");
        for (uint8_t i = 0; i < 16; i++)
        {
            uint8_t src = manualInputMap[i].source;
            Serial.print("    Section ");
            if (i < 9)
                Serial.print(" ");
            Serial.print(i + 1);
            Serial.print(" → ");

            if (src == 0)
                Serial.println("(unassigned)");
            else if (src >= 101 && src <= 108)
            {
                Serial.print("CAN IN");
                Serial.println(src - 100);
            }
            else
            {
                Serial.print("GPIO pin ");
                Serial.println(src);
            }
        }
    }

#elif (SC_OUTPUT_MODE == SC_I2C)
    Serial.println("  Output mode: I2C XL9535");
    initXL9535();

#else
    Serial.println("  Output mode: GPIO Teensy");
    for (uint8_t i = 0; i < NUM_CONFIGURABLE_PINS; i++)
    {
        pinMode(teensy_pins[i], OUTPUT);
        digitalWrite(teensy_pins[i], aogConfig.isRelayActiveHigh ? LOW : HIGH);
    }
#endif

    // ---- Auto/Manual mode source ----
#if (SC_AUTO_MANUAL_INPUT >= 1) && (SC_AUTO_MANUAL_INPUT <= 41)
    pinMode(SC_AUTO_MANUAL_INPUT, INPUT_PULLUP);
    Serial.print("  Auto/Manual source: GPIO Pin ");
    Serial.println(SC_AUTO_MANUAL_INPUT);
#elif (SC_AUTO_MANUAL_INPUT >= 101) && (SC_AUTO_MANUAL_INPUT <= 116)
    Serial.print("  Auto/Manual source: CAN IN");
    Serial.println(SC_AUTO_MANUAL_INPUT - 100);
#else
    Serial.println("  Auto/Manual source: Disabled (always AUTO)");
#endif

    // ---- Init GPIO manual pins ----
    for (uint8_t i = 0; i < 16; i++)
    {
        uint8_t src = manualInputMap[i].source;
        if (src >= 1 && src <= 41)
            pinMode(src, INPUT_PULLUP);
    }

    // ---- AOG mapping ----
    Serial.println("  AOG Pin Mapping:");
    for (uint8_t i = 0; i < NUM_CONFIGURABLE_PINS; i++)
    {
        Serial.print("    Pos ");
        Serial.print(i + 1);
        Serial.print(" fn=");
        Serial.print(pin[i]);
#if (SC_OUTPUT_MODE == SC_CAN)
        Serial.print(" → CAN Relay ");
        Serial.println(i + 1);
#elif (SC_OUTPUT_MODE == SC_I2C)
        Serial.print(" → I2C Relay ");
        Serial.println(i);
#else
        Serial.print(" → Teensy Pin ");
        Serial.println(teensy_pins[i]);
#endif
    }

    // ---- Init states ----
    for (uint8_t i = 0; i < 23; i++)
        relayState[i] = 0;
    raiseTimer = lowerTimer = lastTrigger = 0;
    isRaise = isLower = false;
    onLo = onHi = 0;
    offLo = offHi = 0xFF;

    // ---- Init Master Valve ----
#if (VG_ENABLE == 1)
#if (VG_PIN_RELAY != 0)
    pinMode(VG_PIN_RELAY, OUTPUT);
    applyVGRelay(); // MV closed at startup
#endif
#if (VG_PIN_BUTTON >= 1) && (VG_PIN_BUTTON <= 41)
    pinMode(VG_PIN_BUTTON, INPUT_PULLUP);
    Serial.print("  MV button: GPIO Pin ");
    Serial.println(VG_PIN_BUTTON);
#elif (VG_PIN_BUTTON >= 101) && (VG_PIN_BUTTON <= 116)
    Serial.print("  MV button: CAN IN");
    Serial.println(VG_PIN_BUTTON - 100);
#endif
#if (VG_PIN_RELAY != 0)
    Serial.print("  MV relay: Pin ");
    Serial.println(VG_PIN_RELAY);
#endif
    Serial.print("  MV auto-close delay: ");
    Serial.print(VG_CLOSE_DELAY_MS);
    Serial.println(" ms");
#endif

    Serial.print("  Relay Active High : ");
    Serial.println(aogConfig.isRelayActiveHigh ? "YES" : "NO");
    Serial.println("=== Section Control Ready ===\r\n");
}

//  MAIN LOOP
void sectionControlLoop()
{
    uint32_t currentTime = millis();

    // ---- Passive CAN read (non-blocking, drains buffer every loop) ----
#if (SC_OUTPUT_MODE == SC_CAN)
    pollCANMessages();
#endif

    // ---- CAN input polling: periodic request ----
#if (SC_OUTPUT_MODE == SC_CAN)
    if (currentTime - lastCANInputPoll >= SC_CAN_INPUT_POLL_MS)
    {
        lastCANInputPoll = currentTime;
        readCANInputs();
    }
#endif

    // ---- Read Auto/Manual mode ----
    bool newAutoMode = getAutoManual();

    if (newAutoMode != autoModeIsOn)
    {
        autoModeIsOn = newAutoMode;
        if (!autoModeIsOn)
        {
            relay = relayHi = 0;
            onLo = onHi = 0;
            offLo = offHi = 0xFF;
            Serial.println("[SC] → MANUAL mode");
        }
        else
        {
            Serial.println("[SC] → AUTO mode");
        }
    }
    mainByte = autoModeIsOn ? 1 : 2;

    // ---- 5Hz control loop ----
    if (currentTime - lastHydraulicTime >= HYDRAULIC_LOOP_TIME)
    {
        lastHydraulicTime = currentTime;

        processHydraulicLift();

        if (autoModeIsOn)
            processAutoMode();
        else
            processManualMode();

        processVG(); // Master Valve (timer + button)
        SetRelays();
        sendSectionStatus();
        sendManualAutoStatus();
    }
}

// ============================================================================
//  AUTO MODE
// ============================================================================

void processAutoMode()
{
    onLo = onHi = 0;
    offLo = offHi = 0;
}

// ============================================================================
//  MANUAL MODE - GPIO and/or CAN inputs
// ============================================================================

void processManualMode()
{
    onLo = onHi = 0;
    offLo = offHi = 0;

#if (SC_MANUAL_RELAY_MODE == 0)
    // Mode 0: AOG info only - physical relays stay OFF
    relay = relayHi = 0;
#endif

    for (uint8_t i = 0; i < 16; i++)
    {
        bool switchClosed = false;
        uint8_t src = manualInputMap[i].source;

#if (VG_ENABLE == 1)
        if (vgActive)
        {
            // MV active: read physical inputs in real time
            if (src >= 101 && src <= 108)
                switchClosed = bitRead(canInputStates, src - 101);
            else if (src >= 109 && src <= 116)
                switchClosed = bitRead(canInputStates2, src - 109);
            else if (src >= 1 && src <= 41)
                switchClosed = !digitalRead(src);

            // Update vgForcedOn in real time to reflect current state
            if (i < 8)
                bitWrite(vgForcedOnLo, i, switchClosed ? 1 : 0);
            else
                bitWrite(vgForcedOnHi, i - 8, switchClosed ? 1 : 0);
        }
        // MV inactive: all sections forced OFF (switchClosed stays false)
#else
        if (src >= 101 && src <= 108)
            switchClosed = bitRead(canInputStates, src - 101); // CAN IN1-IN8
        else if (src >= 109 && src <= 116)
            switchClosed = bitRead(canInputStates2, src - 109); // IN9-16
        else if (src >= 1 && src <= 41)
            switchClosed = !digitalRead(src); // GPIO LOW=ON
#endif

        // Fill onLo/offLo/onHi/offHi to inform AOG via PGN 234
        if (i < 8) // Sections 1-8: onLo / offLo
        {
            if (switchClosed)
                bitSet(onLo, i);
            else
                bitSet(offLo, i);
        }
        else // Sections 9-16: onHi / offHi
        {
            uint8_t b = i - 8;
            if (switchClosed)
                bitSet(onHi, b);
            else
                bitSet(offHi, b);
        }

#if (SC_MANUAL_RELAY_MODE == 1)
        // Mode 1: also activate physical relays directly
        if (i < 8)
            bitWrite(relay, i, switchClosed ? 1 : 0);
        else
            bitWrite(relayHi, i - 8, switchClosed ? 1 : 0);
#endif
    }
}

// ============================================================================
//  TURN OFF ALL RELAYS
// ============================================================================

void switchRelaysOff()
{
    relay = relayHi = 0;
    onLo = onHi = 0;
    offLo = offHi = 0xFF;
}

// ============================================================================
//  MASTER VALVE - Apply physical relay state
// ============================================================================

void applyVGRelay()
{
#if (VG_ENABLE == 1)
#if (VG_PIN_RELAY != 0)
    bool phys = aogConfig.isRelayActiveHigh ? vgIsOpen : !vgIsOpen;
    digitalWrite(VG_PIN_RELAY, phys ? HIGH : LOW);
#endif
#endif
}

// Open the MV (and cancel the auto-close timer)
void openVG()
{
#if (VG_ENABLE == 1)
    if (!vgIsOpen)
    {
        vgIsOpen = true;
        vgAllOffTimerActive = false;
        applyVGRelay();
        Serial.println("[VG] → OPEN");
    }
#endif
}

// Close the MV
void closeVG()
{
#if (VG_ENABLE == 1)
    if (vgIsOpen)
    {
        vgIsOpen = false;
        vgAllOffTimerActive = false;
        applyVGRelay();
        Serial.println("[VG] → CLOSED");
    }
#endif
}

// ============================================================================
//  MASTER VALVE - Automatic logic (called every 5Hz loop)
// ============================================================================

void processVG()
{
#if (VG_ENABLE == 1)
    uint32_t now = millis();

    // --- Direct MV ON/OFF button (switch, MANUAL mode only) ---
    // GPIO: LOW=ON, HIGH=OFF — CAN: active=ON, inactive=OFF
#if (VG_PIN_BUTTON != 0)
    bool btnState = false;
#if (VG_PIN_BUTTON >= 1) && (VG_PIN_BUTTON <= 41)
    btnState = !digitalRead(VG_PIN_BUTTON); // GPIO : LOW=ON
#elif (VG_PIN_BUTTON >= 101) && (VG_PIN_BUTTON <= 108)
    btnState = bitRead(canInputStates, VG_PIN_BUTTON - 101); // CAN IN1-IN8
#elif (VG_PIN_BUTTON >= 109) && (VG_PIN_BUTTON <= 116)
    btnState = bitRead(canInputStates2, VG_PIN_BUTTON - 109); // CAN IN9-IN16
#endif

    if (!autoModeIsOn) // Button active in MANUAL mode only
    {
        if (btnState && !vgActive)
        {
            // Transition OFF → ON
            vgForcedOnLo = 0;
            vgForcedOnHi = 0;
            vgActive = true;
            Serial.println("[VG] ON");
            openVG();
        }
        else if (!btnState && vgActive)
        {
            // Transition ON → OFF
            vgActive = false;
            vgForcedOnLo = 0;
            vgForcedOnHi = 0;
            Serial.println("[VG] OFF");
            closeVG();
        }
    }
    else
    {
        // In AUTO mode: ignore button, reset MV state
        if (vgActive)
        {
            vgActive = false;
            vgForcedOnLo = 0;
            vgForcedOnHi = 0;
        }
    }
#endif

    // --- Auto-close if all sections OFF for VG_CLOSE_DELAY_MS (AUTO mode only) ---
    if (autoModeIsOn)
    {
        bool anySectionOn = (relay != 0 || relayHi != 0);

        if (anySectionOn)
        {
            // At least one section ON → open MV immediately + reset timer
            vgAllOffTimerActive = false;
            openVG();
        }
        else
        {
            // All sections OFF
            if (vgIsOpen)
            {
                if (!vgAllOffTimerActive)
                {
                    vgAllOffTimerActive = true;
                    vgAllOffSince = now;
                }
                else if (now - vgAllOffSince >= VG_CLOSE_DELAY_MS)
                {
                    Serial.println("[VG] All sections OFF for 5s -> auto-close");
                    closeVG();
                }
            }
            else
            {
                vgAllOffTimerActive = false;
            }
        }
    }
    else
    {
        // In MANUAL mode: reset timer to avoid spurious close when returning to AUTO
        vgAllOffTimerActive = false;
    }
#endif
}

// ============================================================================
//  HYDRAULIQUE
// ============================================================================

void processHydraulicLift()
{
    if (hydLift != lastTrigger && (hydLift == 1 || hydLift == 2))
    {
        lastTrigger = hydLift;
        lowerTimer = raiseTimer = 0;
        if (hydLift == 1)
            lowerTimer = aogConfig.lowerTime * 5;
        if (hydLift == 2)
            raiseTimer = aogConfig.raiseTime * 5;
    }

    if (raiseTimer)
    {
        raiseTimer--;
        lowerTimer = 0;
    }
    if (lowerTimer)
        lowerTimer--;

    if (hydLift != 1 && hydLift != 2)
    {
        lowerTimer = raiseTimer = 0;
        lastTrigger = 0;
    }

    if (aogConfig.isRelayActiveHigh)
    {
        isLower = isRaise = false;
        if (lowerTimer)
            isLower = true;
        if (raiseTimer)
            isRaise = true;
    }
    else
    {
        isLower = isRaise = true;
        if (lowerTimer)
            isLower = false;
        if (raiseTimer)
            isRaise = false;
    }
}

//  SET RELAYS - Apply states (GPIO / I2C / CAN)
void SetRelays()
{
    // Build logical state array from relay/relayHi/tram/hydro
    for (uint8_t i = 0; i < 8; i++)
    {
        relayState[i] = bitRead(relay, i);
        relayState[i + 8] = bitRead(relayHi, i);
    }
    relayState[16] = isLower;
    relayState[17] = isRaise;
    relayState[18] = bitRead(tram, 0); // Tram Right
    relayState[19] = bitRead(tram, 1); // Tram Left
    relayState[20] = 0;                // GeoStop

    // Inline helper: logical state → physical state according to active high/low
    // reads from relayState[] based on assigned function
    auto getPhysical = [](uint8_t fn) -> bool
    {
        bool logical = false;
        if (fn >= 1 && fn <= 16)
            logical = relayState[fn - 1];
        else if (fn == 17)
            logical = relayState[16]; // Hyd Up
        else if (fn == 18)
            logical = relayState[17]; // Hyd Down
        else if (fn == 19)
            logical = relayState[18]; // Tram Right
        else if (fn == 20)
            logical = relayState[19]; // Tram Left
        else if (fn == 21)
            logical = relayState[20]; // GeoStop
        return aogConfig.isRelayActiveHigh ? logical : !logical;
    };

// ---- CAN ----
#if (SC_OUTPUT_MODE == SC_CAN)
    uint8_t canBuf[2] = {0x00, 0x00};
    for (uint8_t i = 0; i < 16; i++) // 16 CAN relays max (independent of NUM_CONFIGURABLE_PINS)
    {
        uint8_t fn = pin[i];
        if (fn == 0 || fn > 21)
            continue;
        bool phys = getPhysical(fn);
        if (i < 8)
            bitWrite(canBuf[0], i, phys); // Relays 1-8  → buf[0]
        else
            bitWrite(canBuf[1], i - 8, phys); // Relays 9-16 → buf[1]
    }
    updateAllCANRelays(canBuf[0], canBuf[1]);

// ---- I2C ----
#elif (SC_OUTPUT_MODE == SC_I2C)
    for (uint8_t i = 0; i < 16; i++) // 16 I2C relays max (independent of NUM_CONFIGURABLE_PINS)
    {
        uint8_t fn = pin[i];
        if (fn == 0 || fn > 21)
            continue;
        // setI2CRelay already applies active high/low internally
        bool logical = false;
        if (fn >= 1 && fn <= 16)
            logical = relayState[fn - 1];
        else if (fn == 17)
            logical = relayState[16];
        else if (fn == 18)
            logical = relayState[17];
        else if (fn == 19)
            logical = relayState[18];
        else if (fn == 20)
            logical = relayState[19];
        else if (fn == 21)
            logical = relayState[20];
        setI2CRelay(i, logical);
    }
    updateAllI2CRelays();

// ---- GPIO ----
#else
    for (uint8_t i = 0; i < NUM_CONFIGURABLE_PINS; i++)
    {
        uint8_t fn = pin[i];
        if (fn == 0 || fn > 21)
            continue;
        digitalWrite(teensy_pins[i], getPhysical(fn) ? HIGH : LOW);
    }
#endif
}

// ============================================================================
//  SEND STATUS TO AgOpenGPS (PGN 237)
// ============================================================================

void sendSectionStatus()
{
    PGN_237[5] = relay;
    PGN_237[6] = relayHi;
    PGN_237[7] = tram;
    PGN_237[8] = isRaise ? 2 : (isLower ? 1 : 0);

    int16_t CK_A = 0;
    for (uint8_t i = 2; i < PGN_237_Size; i++)
        CK_A += PGN_237[i];
    PGN_237[PGN_237_Size] = CK_A;

    Udp.beginPacket(ipDestination, AOGPort);
    Udp.write(PGN_237, sizeof(PGN_237));
    Udp.endPacket();
}

// ============================================================================
//  SEND MANUAL/AUTO STATUS TO AgOpenGPS (PGN 234)
// ============================================================================

void sendManualAutoStatus()
{
    PGN_234[5] = mainByte; // 1=Auto 2=Manual
    PGN_234[6] = 0;
    PGN_234[7] = 0;
    PGN_234[8] = 0;
    PGN_234[9] = onLo;
    PGN_234[10] = offLo;
    PGN_234[11] = onHi;
    PGN_234[12] = offHi;

    int16_t CK_A = 0;
    for (uint8_t i = 2; i < PGN_234_Size; i++)
        CK_A += PGN_234[i];
    PGN_234[PGN_234_Size] = CK_A;

    Udp.beginPacket(ipDestination, AOGPort);
    Udp.write(PGN_234, sizeof(PGN_234));
    Udp.endPacket();
}

// ============================================================================
//  UPDATE MAPPING (from AgOpenGPS - PGN 200)
// ============================================================================

void updatePinMapping(uint8_t *udpData)
{
    for (uint8_t i = 0; i < 24; i++)
        pin[i] = udpData[i + 5];
    EEPROM.put(120, pin);
}

// ============================================================================
//  RECEIVE PGN 239 (Machine Data from AgOpenGPS)
// ============================================================================

void handlePGN239(uint8_t *udpData)
{
    uTurn = udpData[5];
    gpsSpeed = (float)udpData[6] * 0.1f;
    hydLift = udpData[7];
    tram = udpData[8];

    uint8_t receivedRelay = udpData[11];
    uint8_t receivedRelayHi = udpData[12];

    // In MANUAL mode, ignore AOG section commands
    if (autoModeIsOn)
    {
        relay = receivedRelay;
        relayHi = receivedRelayHi;
    }
}
