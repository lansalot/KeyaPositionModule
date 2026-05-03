//FlorianT NMEA OUT - Version with Configuration Menu
// File to generate a new NMEA 0183 frame corrected for configurable roll/pitch offset

#include <math.h>

/************************* NMEA 0183 User Settings *************************/
// Antenna height for roll/pitch correction (Antenna Phase Center Height)
float ANTENNA_HEIGHT_M = 3.0; 

// NMEA 0183 output serial port configuration
extern Stream* NmeaOutputSerial;
long baudNMEA = 38400;       

// NMEA messages to send (configurable)
bool SEND_GPGGA = true;
bool SEND_GPVTG = true;
bool SEND_GPRMC = true;
bool SEND_GPZDA = true;

// Independent send frequencies for each message (in Hz)
float GPGGA_FREQUENCY_HZ = 10.0;  // 10 Hz
float GPVTG_FREQUENCY_HZ = 10.0;  // 10 Hz
float GPRMC_FREQUENCY_HZ = 10.0;  // 10 Hz
float GPZDA_FREQUENCY_HZ = 1.0;   // 1 Hz

// Interval variables (will be recalculated)
uint32_t GPGGA_INTERVAL_MS = 100;
uint32_t GPVTG_INTERVAL_MS = 100;
uint32_t GPRMC_INTERVAL_MS = 100;
uint32_t GPZDA_INTERVAL_MS = 1000;

/************************* Configuration Menu *************************/
bool menuActive = false;
String inputBuffer = "";

void recalculateIntervals() {
    GPGGA_INTERVAL_MS = (uint32_t)(1000.0 / GPGGA_FREQUENCY_HZ);
    GPVTG_INTERVAL_MS = (uint32_t)(1000.0 / GPVTG_FREQUENCY_HZ);
    GPRMC_INTERVAL_MS = (uint32_t)(1000.0 / GPRMC_FREQUENCY_HZ);
    GPZDA_INTERVAL_MS = (uint32_t)(1000.0 / GPZDA_FREQUENCY_HZ);
}

void displayMenu() {
    Serial.println(F("\n╔════════════════════════════════════════════════╗"));
    Serial.println(F("║      NMEA OUTPUT CONFIGURATION MENU            ║"));
    Serial.println(F("╠════════════════════════════════════════════════╣"));
    Serial.println(F("║  1. Enable/Disable messages                    ║"));
    Serial.println(F("║  2. Set frequencies (Hz)                       ║"));
    Serial.println(F("║  3. Set Baudrate                               ║"));
    Serial.println(F("║  4. Set antenna height                         ║"));
    Serial.println(F("║  5. Show current configuration                 ║"));
    Serial.println(F("║  6. Save config (EEPROM)                       ║"));
    Serial.println(F("║  7. Load config (EEPROM)                       ║"));
    Serial.println(F("║  0. Exit menu                                  ║"));
    Serial.println(F("╚════════════════════════════════════════════════╝"));
    Serial.print(F("Choice: "));
}

void displayCurrentConfig() {
    Serial.println(F("\n╔════════════════════════════════════════════════╗"));
    Serial.println(F("║      CURRENT CONFIGURATION                     ║"));
    Serial.println(F("╠════════════════════════════════════════════════╣"));
    Serial.print(F("║ Baudrate: ")); Serial.print(baudNMEA); Serial.println(F(" bps"));
    Serial.print(F("║ Antenna height: ")); Serial.print(ANTENNA_HEIGHT_M, 2); Serial.println(F(" m"));
    Serial.println(F("║                                                ║"));
    Serial.println(F("║ Active messages:                               ║"));
    Serial.print(F("║   GPGGA: ")); Serial.print(SEND_GPGGA ? "ON " : "OFF"); 
    Serial.print(F(" @ ")); Serial.print(GPGGA_FREQUENCY_HZ, 1); Serial.println(F(" Hz"));
    Serial.print(F("║   GPVTG: ")); Serial.print(SEND_GPVTG ? "ON " : "OFF"); 
    Serial.print(F(" @ ")); Serial.print(GPVTG_FREQUENCY_HZ, 1); Serial.println(F(" Hz"));
    Serial.print(F("║   GPRMC: ")); Serial.print(SEND_GPRMC ? "ON " : "OFF"); 
    Serial.print(F(" @ ")); Serial.print(GPRMC_FREQUENCY_HZ, 1); Serial.println(F(" Hz"));
    Serial.print(F("║   GPZDA: ")); Serial.print(SEND_GPZDA ? "ON " : "OFF"); 
    Serial.print(F(" @ ")); Serial.print(GPZDA_FREQUENCY_HZ, 1); Serial.println(F(" Hz"));
    Serial.println(F("╚════════════════════════════════════════════════╝\n"));
}

void submenuMessages() {
    Serial.println(F("\n=== ENABLE/DISABLE MESSAGES ==="));
    Serial.println(F("1. GPGGA (Position) - Currently: ") + String(SEND_GPGGA ? "ON" : "OFF"));
    Serial.println(F("2. GPVTG (Speed)    - Currently: ") + String(SEND_GPVTG ? "ON" : "OFF"));
    Serial.println(F("3. GPRMC (Position+Speed) - Currently: ") + String(SEND_GPRMC ? "ON" : "OFF"));
    Serial.println(F("4. GPZDA (Date/Time) - Currently: ") + String(SEND_GPZDA ? "ON" : "OFF"));
    Serial.println(F("0. Back"));
    Serial.print(F("Choice: "));
    
    while (!Serial.available()) { delay(10); }
    int choice = Serial.parseInt();
    while (Serial.available()) Serial.read();
    
    switch(choice) {
        case 1: SEND_GPGGA = !SEND_GPGGA; Serial.println(SEND_GPGGA ? "GPGGA ON" : "GPGGA OFF"); break;
        case 2: SEND_GPVTG = !SEND_GPVTG; Serial.println(SEND_GPVTG ? "GPVTG ON" : "GPVTG OFF"); break;
        case 3: SEND_GPRMC = !SEND_GPRMC; Serial.println(SEND_GPRMC ? "GPRMC ON" : "GPRMC OFF"); break;
        case 4: SEND_GPZDA = !SEND_GPZDA; Serial.println(SEND_GPZDA ? "GPZDA ON" : "GPZDA OFF"); break;
    }
}

void submenuFrequencies() {
    Serial.println(F("\n=== SET FREQUENCIES (Hz) ==="));
    Serial.println(F("1. GPGGA - Currently: ") + String(GPGGA_FREQUENCY_HZ, 1) + F(" Hz"));
    Serial.println(F("2. GPVTG - Currently: ") + String(GPVTG_FREQUENCY_HZ, 1) + F(" Hz"));
    Serial.println(F("3. GPRMC - Currently: ") + String(GPRMC_FREQUENCY_HZ, 1) + F(" Hz"));
    Serial.println(F("4. GPZDA - Currently: ") + String(GPZDA_FREQUENCY_HZ, 1) + F(" Hz"));
    Serial.println(F("0. Back"));
    Serial.print(F("Choice: "));
    
    while (!Serial.available()) { delay(10); }
    int choice = Serial.parseInt();
    while (Serial.available()) Serial.read();
    
    if (choice >= 1 && choice <= 4) {
        Serial.print(F("New frequency (0.1 to 50 Hz): "));
        while (!Serial.available()) { delay(10); }
        float newFreq = Serial.parseFloat();
        while (Serial.available()) Serial.read();
        
        if (newFreq >= 0.1 && newFreq <= 50.0) {
            switch(choice) {
                case 1: GPGGA_FREQUENCY_HZ = newFreq; break;
                case 2: GPVTG_FREQUENCY_HZ = newFreq; break;
                case 3: GPRMC_FREQUENCY_HZ = newFreq; break;
                case 4: GPZDA_FREQUENCY_HZ = newFreq; break;
            }
            recalculateIntervals();
            Serial.println(F("Frequency updated!"));
        } else {
            Serial.println(F("Invalid value!"));
        }
    }
}

void submenuBaudrate() {
    Serial.println(F("\n=== SET BAUDRATE ==="));
    Serial.println(F("Current baudrate: ") + String(baudNMEA));
    Serial.println(F("\nStandard baudrates:"));
    Serial.println(F("1. 4800"));
    Serial.println(F("2. 9600"));
    Serial.println(F("3. 19200"));
    Serial.println(F("4. 38400"));
    Serial.println(F("5. 57600"));
    Serial.println(F("6. 115200"));
    Serial.println(F("7. Custom"));
    Serial.println(F("0. Back"));
    Serial.print(F("Choice: "));
    
    while (!Serial.available()) { delay(10); }
    int choice = Serial.parseInt();
    while (Serial.available()) Serial.read();
    
    long newBaud = 0;
    switch(choice) {
        case 1: newBaud = 4800; break;
        case 2: newBaud = 9600; break;
        case 3: newBaud = 19200; break;
        case 4: newBaud = 38400; break;
        case 5: newBaud = 57600; break;
        case 6: newBaud = 115200; break;
        case 7:
            Serial.print(F("Enter custom baudrate: "));
            while (!Serial.available()) { delay(10); }
            newBaud = Serial.parseInt();
            while (Serial.available()) Serial.read();
            break;
    }
    
    if (newBaud > 0) {
        baudNMEA = newBaud;
        if (NmeaOutputSerial != NULL && NmeaOutputSerial != &Serial) {
            ((HardwareSerial*)NmeaOutputSerial)->end();
            ((HardwareSerial*)NmeaOutputSerial)->begin(baudNMEA);
        }
        Serial.println(F("Baudrate updated!"));
    }
}

void submenuAntennaHeight() {
    Serial.println(F("\n=== ANTENNA HEIGHT ==="));
    Serial.print(F("Current height: ")); Serial.print(ANTENNA_HEIGHT_M, 2); Serial.println(F(" m"));
    Serial.print(F("New height (0.1 to 10.0 m): "));
    
    while (!Serial.available()) { delay(10); }
    float newHeight = Serial.parseFloat();
    while (Serial.available()) Serial.read();
    
    if (newHeight >= 0.1 && newHeight <= 10.0) {
        ANTENNA_HEIGHT_M = newHeight;
        Serial.println(F("Height updated!"));
    } else {
        Serial.println(F("Invalid value!"));
    }
}

#define EEPROM_NMEA_ADDR 500  // Start address in EEPROM

struct NmeaConfig {
    uint16_t signature = 0xABCD;  // Signature to verify if config is valid
    float antennaHeight;
    long baudrate;
    bool sendGGA;
    bool sendVTG;
    bool sendRMC;
    bool sendZDA;
    float freqGGA;
    float freqVTG;
    float freqRMC;
    float freqZDA;
};

void saveConfigToEEPROM() {
    NmeaConfig config;
    config.antennaHeight = ANTENNA_HEIGHT_M;
    config.baudrate = baudNMEA;
    config.sendGGA = SEND_GPGGA;
    config.sendVTG = SEND_GPVTG;
    config.sendRMC = SEND_GPRMC;
    config.sendZDA = SEND_GPZDA;
    config.freqGGA = GPGGA_FREQUENCY_HZ;
    config.freqVTG = GPVTG_FREQUENCY_HZ;
    config.freqRMC = GPRMC_FREQUENCY_HZ;
    config.freqZDA = GPZDA_FREQUENCY_HZ;
    
    EEPROM.put(EEPROM_NMEA_ADDR, config);
    Serial.println(F("\xE2\x9C\x93 Configuration saved to EEPROM!"));
}

void loadConfigFromEEPROM() {
    NmeaConfig config;
    EEPROM.get(EEPROM_NMEA_ADDR, config);
    
    if (config.signature == 0xABCD) {
        ANTENNA_HEIGHT_M = config.antennaHeight;
        baudNMEA = config.baudrate;
        SEND_GPGGA = config.sendGGA;
        SEND_GPVTG = config.sendVTG;
        SEND_GPRMC = config.sendRMC;
        SEND_GPZDA = config.sendZDA;
        GPGGA_FREQUENCY_HZ = config.freqGGA;
        GPVTG_FREQUENCY_HZ = config.freqVTG;
        GPRMC_FREQUENCY_HZ = config.freqRMC;
        GPZDA_FREQUENCY_HZ = config.freqZDA;
        
        recalculateIntervals();
        
        if (NmeaOutputSerial != NULL && NmeaOutputSerial != &Serial) {
            ((HardwareSerial*)NmeaOutputSerial)->end();
            ((HardwareSerial*)NmeaOutputSerial)->begin(baudNMEA);
        }
        
        Serial.println(F("\xE2\x9C\x93 Configuration loaded from EEPROM!"));
    } else {
        Serial.println(F("\xE2\x9C\x97 No valid configuration found in EEPROM"));
    }
}

void processMenu() {
    if (!menuActive) return;
    
    if (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                int choice = inputBuffer.toInt();
                inputBuffer = "";
                
                Serial.println();
                
                switch(choice) {
                    case 0:
                        menuActive = false;
                        Serial.println(F("Menu closed. Press 'm' to reopen."));
                        break;
                    case 1:
                        submenuMessages();
                        displayMenu();
                        break;
                    case 2:
                        submenuFrequencies();
                        displayMenu();
                        break;
                    case 3:
                        submenuBaudrate();
                        displayMenu();
                        break;
                    case 4:
                        submenuAntennaHeight();
                        displayMenu();
                        break;
                    case 5:
                        displayCurrentConfig();
                        displayMenu();
                        break;
                    case 6:
                        saveConfigToEEPROM();
                        displayMenu();
                        break;
                    case 7:
                        loadConfigFromEEPROM();
                        displayMenu();
                        break;
                    default:
                        Serial.println(F("Invalid choice!"));
                        displayMenu();
                }
            }
        } else if (c >= '0' && c <= '9') {
            inputBuffer += c;
            Serial.print(c);
        }
    }
}

void checkMenuActivation() {
    if (Serial.available()) {
        char c = Serial.peek();
        if (c == 'm' || c == 'M') {
            Serial.read();
            menuActive = true;
            displayMenu();
        }
    }
}

/************************* Global Variables and Declarations *************************/
// [Rest of code remains identical...]
extern const char* asciiHex;
extern char fixTime[12];
extern char latitude[15];
extern char latNS[3];
extern char longitude[15];
extern char lonEW[3];
extern char fixQuality[2];
extern char numSats[4];
extern char HDOP[5];
extern char altitude[12];
extern char ageDGPS[10];
extern char rmcDate[7];
extern char rmcMagVar[6];
extern char rmcMagEW[3];
extern char vtgHeading[12]; 
extern char speedKnots[10];
extern char imuRoll[6];     
extern char imuPitch[6];    
extern bool GGA_Available;

elapsedMillis nmeaGgaTimer = 0;
elapsedMillis nmeaVtgTimer = 0;
elapsedMillis nmeaRmcTimer = 0;
elapsedMillis nmeaZdaTimer = 0;

char nmea0183Buffer[100]; 
const float EARTH_RADIUS_M = 6371000.0;

/************************* Utility Functions *************************/

// Calculates and appends the NMEA 0183 checksum to the 'nmea' string
void BuildChecksum0183(char *nmea)
{
  uint8_t sum = 0;
  char *p = nmea;
  
  if (*p == '$') p++; 

  while (*p != '\0')
  {
    sum ^= *p;
    p++;
  }
  
  *p++ = '*'; 

  uint8_t chk = (sum >> 4); 
  *p++ = asciiHex[chk];

  chk = (sum & 0x0F);
  *p++ = asciiHex[chk];

  *p++ = '\r';
  *p++ = '\n';
  *p = '\0'; 
}

// Converts NMEA position (ddmm.mmmm) to decimal degrees (dd.ddddd)
double nmeaToDecimal(char* nmea_str, char ns_ew_char)
{
    char* endptr;
    double ddmm = strtod(nmea_str, &endptr);
    if (ddmm == 0.0) return 0.0;
    
    double degrees = floor(ddmm / 100.0);
    double minutes = ddmm - (degrees * 100.0);
    double decimal_degrees = degrees + (minutes / 60.0);
    
    if (ns_ew_char == 'S' || ns_ew_char == 'W') {
        return -decimal_degrees;
    }
    return decimal_degrees;
}

// Converts decimal position (dd.ddddd) to NMEA format (ddmm.mmmm) and sets N/S or E/W marker
void decimalToNMEA(double decimal_degrees, char* nmea_buf, char* ns_ew_buf, int min_prec, PositionType pos_type)
{
    // For altitude, use the standard simplified format
    if (min_prec == 1) { 
        dtostrf(decimal_degrees, 6, min_prec, nmea_buf);
        return;
    }
    
    // --- 1. Determine direction (N/S or E/W) ---
    if (decimal_degrees < 0) {
        *ns_ew_buf = (pos_type == LATITUDE) ? 'S' : 'W'; 
        decimal_degrees = -decimal_degrees;
    } else {
        *ns_ew_buf = (pos_type == LATITUDE) ? 'N' : 'E'; 
    }
    *(ns_ew_buf + 1) = '\0';

    // --- 2. Split parts (degrees, whole minutes, fractional minutes) ---
    int deg = (int)floor(decimal_degrees); // Whole degrees (e.g. 1 for 1.86°)
    double minutes_float = (decimal_degrees - deg) * 60.0; // Minutes with decimals (e.g. 51.6')
    
    int min_int = (int)floor(minutes_float); // Whole minutes (e.g. 51)
    
    // Calculate fractional minutes (4 decimal places for NMEA)
    // min_prec is 4 for Lat/Lon
    int min_frac = (int)round((minutes_float - min_int) * pow(10.0, min_prec)); 
    
    // --- 3. Format with sprintf to guarantee leading zeros ---
    // This method is the only one guaranteeing leading zeros (%0xd)

    if (pos_type == LATITUDE) 
    {
        // LAT: ddmm.mmmm
        // %02d : 2-digit degrees with leading zero if needed (e.g. 43)
        // %02d : 2-digit minutes with leading zero if needed (e.g. 46)
        // %04d : 4 decimal minutes with leading zeros if needed
        sprintf(nmea_buf, "%02d%02d.%04d", deg, min_int, min_frac);
    } 
    else // LONGITUDE
    { 
        // LON: dddmm.mmmm
        // %03d : 3-digit degrees with leading zeros (e.g. 001) <--- KEY FIX
        // %02d : 2-digit minutes
        // %04d : 4 decimals
        sprintf(nmea_buf, "%03d%02d.%07d", deg, min_int, min_frac);
    }
}

/************************* Roll/Pitch Correction *************************/

// Applies position (Lat/Lon) and altitude correction due to roll and pitch.
double applyRollPitchCorrection(double& lat_deg, double& lon_deg, float heading_deg)
{
    // Roll and Pitch in radians
    char* endptr;

    // --- Conversion with division by 10 (REQUIRED because imuRoll/Pitch is angle * 10) ---
    float roll_deg_x10 = strtod(imuRoll, &endptr);
    float roll_deg = roll_deg_x10 / 10.0; // Correction: angle in real degrees
    float roll_rad = roll_deg * (PI / 180.0);
    
    float pitch_deg_x10 = strtod(imuPitch, &endptr);
    float pitch_deg = pitch_deg_x10 / 10.0; // Correction: angle in real degrees
    float pitch_rad = pitch_deg * (PI / 180.0);

    
    // Heading in radians
    float heading_deg_clamped = strtod(vtgHeading, &endptr);
    float heading_rad = heading_deg_clamped * (PI / 180.0);

    // 1. Altitude correction (vertical projection)
    double alt_correction_m = ANTENNA_HEIGHT_M * (1.0 - (cos(roll_rad) * cos(pitch_rad)));
    
    // 2. Horizontal offset (lateral and longitudinal)
    double dY_m = ANTENNA_HEIGHT_M * sin(roll_rad); // Lateral offset (Cross-track)
    double dX_m = ANTENNA_HEIGHT_M * sin(pitch_rad); // Longitudinal offset (Along-track)

    // North (dN_m) and East (dE_m) components of the offset
    double dN_m = (dY_m * sin(heading_rad)) + (dX_m * cos(heading_rad));
    double dE_m = (-dY_m * cos(heading_rad)) + (dX_m * sin(heading_rad));

    // 3. Convert meters to Lat/Lon degrees
    double meters_per_deg_lat = EARTH_RADIUS_M * (PI / 180.0); 
    double meters_per_deg_lon = meters_per_deg_lat * cos(lat_deg * (PI / 180.0));

    double dLat_deg = dN_m / meters_per_deg_lat;
    double dLon_deg = dE_m / meters_per_deg_lon;

    // 4. Apply correction: SUBTRACT the offset
    lat_deg -= dLat_deg; 
    lon_deg -= dLon_deg; 
    
    return alt_correction_m;
}

/************************* NMEA Frame Builders *************************/

// Builds and sends the GPGGA frame (corrected)
void BuildGGA0183()
{
    if (!GGA_Available || !SEND_GPGGA) return;
    
    char* endptr;
    
    // 1. Prepare data
    double lat_deg = nmeaToDecimal(latitude, latNS[0]);  // Get first character
    double lon_deg = nmeaToDecimal(longitude, lonEW[0]); // Get first character
    double alt_m = strtod(altitude, &endptr);
    float heading_deg = strtod(vtgHeading, &endptr);
    
    // 2. Apply correction
    double alt_correction_m = applyRollPitchCorrection(lat_deg, lon_deg, heading_deg);
    double corrected_alt_m = alt_m - alt_correction_m;

    // 3. Convert corrected data to NMEA format
    char corrected_lat_nmea[15];
    char corrected_lon_nmea[15];
    char corrected_latNS[3];
    char corrected_lonEW[3];
    char corrected_alt_str[12];
    
    // Use LATITUDE and LONGITUDE to correct N/S/E/W direction + choose number of decimals
    decimalToNMEA(lat_deg, corrected_lat_nmea, corrected_latNS, 7, LATITUDE); 
    decimalToNMEA(lon_deg, corrected_lon_nmea, corrected_lonEW, 7, LONGITUDE); 
    decimalToNMEA(corrected_alt_m, corrected_alt_str, NULL, 1, LATITUDE); 

    // 4. Build frame
    strcpy(nmea0183Buffer, "$GPGGA,");
    strcat(nmea0183Buffer, fixTime);         strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, corrected_lat_nmea); strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, corrected_latNS);  strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, corrected_lon_nmea); strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, corrected_lonEW);  strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, fixQuality);       strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, numSats);          strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, HDOP);             strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, corrected_alt_str); strcat(nmea0183Buffer, ",M,");
    strcat(nmea0183Buffer, "46.9");           strcat(nmea0183Buffer, ",M,"); 
    strcat(nmea0183Buffer, ageDGPS);          strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, "");             
    
    // 5. Checksum and send
    BuildChecksum0183(nmea0183Buffer);
    NmeaOutputSerial->print(nmea0183Buffer);
}

// Builds and sends the GPVTG frame
void BuildVTG0183()
{
    if (!GGA_Available || !SEND_GPVTG) return;
    
    char* endptr;
    
    // Convert knots (speedKnots) to km/h (speedKnots * 1.852)
    char speedKph[10];
    float kph = strtod(speedKnots, &endptr) * 1.852;
    dtostrf(kph, 4, 1, speedKph);
    
    // 1. Build frame
    strcpy(nmea0183Buffer, "$GPVTG,");
    strcat(nmea0183Buffer, vtgHeading);       strcat(nmea0183Buffer, ",T,"); 
    strcat(nmea0183Buffer, "");               strcat(nmea0183Buffer, ",M,"); 
    strcat(nmea0183Buffer, speedKnots);       strcat(nmea0183Buffer, ",N,"); 
    strcat(nmea0183Buffer, speedKph);         strcat(nmea0183Buffer, ",K,"); 
    strcat(nmea0183Buffer, "A");              strcat(nmea0183Buffer, ","); 
    
    // 2. Checksum and send
    BuildChecksum0183(nmea0183Buffer);
    NmeaOutputSerial->print(nmea0183Buffer);
}



void getGpsDate(char* date_buf)
{
    // Use the real date extracted by RMC_Handler
    if (rmcDate[0] != '\0') { 
        strcpy(date_buf, rmcDate);
    } else {
        // Fall back to static date if no RMC has been received
        strcpy(date_buf, "010123"); 
    }
}

// Builds and sends the GPRMC frame (corrected)
void BuildRMC0183()
{
    if (!GGA_Available || !SEND_GPRMC) return;

    char* endptr;
    
    // 1. Prepare data (same as for GGA)
    double lat_deg = nmeaToDecimal(latitude, latNS[0]);  // Get first character
    double lon_deg = nmeaToDecimal(longitude, lonEW[0]); // Get first character
    float heading_deg = strtod(vtgHeading, &endptr);
    
    // 2. Apply correction (only position is corrected for RMC)
    // Altitude is ignored
    applyRollPitchCorrection(lat_deg, lon_deg, heading_deg); // lat/lon correction in-place

    // 3. Convert corrected data to NMEA format
    char corrected_lat_nmea[18];
    char corrected_lon_nmea[18];
    char corrected_latNS[3];
    char corrected_lonEW[3];
    char dateToSend[7]; // Local buffer for the date
    
    decimalToNMEA(lat_deg, corrected_lat_nmea, corrected_latNS, 7, LATITUDE);
    decimalToNMEA(lon_deg, corrected_lon_nmea, corrected_lonEW, 7, LONGITUDE); 
    getGpsDate(dateToSend); // Call updated function

    // 4. Build frame
    strcpy(nmea0183Buffer, "$GPRMC,"); // Use GP as output Talker ID
    strcat(nmea0183Buffer, fixTime);        strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, "A");            strcat(nmea0183Buffer, ","); 
    strcat(nmea0183Buffer, corrected_lat_nmea); strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, corrected_latNS);  strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, corrected_lon_nmea); strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, corrected_lonEW);  strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, speedKnots);     strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, vtgHeading);     strcat(nmea0183Buffer, ","); 
    strcat(nmea0183Buffer, dateToSend);     strcat(nmea0183Buffer, ",");
    
    // Extracted magnetic variation (Field 10 & 11)
    strcat(nmea0183Buffer, rmcMagVar);      strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, rmcMagEW);       
    
    // Mode Indicator (Field 12, NMEA 4.10) - Simulates D, R, A, or N
    strcat(nmea0183Buffer, ",");
    if (fixQuality[0] == '4' || fixQuality[0] == '5') {
        strcat(nmea0183Buffer, "R"); // R = RTK Fixed/Float
    } else if (fixQuality[0] == '2') {
        strcat(nmea0183Buffer, "D"); // D = Differential (DGPS)
    } else if (fixQuality[0] == '1') {
        strcat(nmea0183Buffer, "A"); // A = Autonomous
    } else {
        strcat(nmea0183Buffer, "N"); // N = Data not valid
    }
    // End of frame

    // 5. Checksum and send
    BuildChecksum0183(nmea0183Buffer);
    NmeaOutputSerial->print(nmea0183Buffer);
}

// Builds and sends the GPZDA frame
void BuildZDA0183()
{
    if (!GGA_Available || !SEND_GPZDA) return;

    char dateToSend[7];
    getGpsDate(dateToSend); // Real date (DDMMYY)
    
    // rmcDate is in ddMMyy format:
    char day[3] = {dateToSend[0], dateToSend[1], '\0'};
    char month[3] = {dateToSend[2], dateToSend[3], '\0'};
    char year[5] = {'2', '0', dateToSend[4], dateToSend[5], '\0'}; 
    
    // Corresponds to UTC+2 (summer time / CEST).
    const char* LOCAL_HOUR_OFFSET = "02"; // Set offset here (00 for UTC)
    const char* LOCAL_MINUTE_OFFSET = "00"; 

    // 1. Build frame
    strcpy(nmea0183Buffer, "$GPZDA,");
    strcat(nmea0183Buffer, fixTime);    strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, day);        strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, month);      strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, year);       strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, LOCAL_HOUR_OFFSET); strcat(nmea0183Buffer, ","); 
    strcat(nmea0183Buffer, LOCAL_MINUTE_OFFSET);                                   

    // 2. Checksum and send
    BuildChecksum0183(nmea0183Buffer);
    NmeaOutputSerial->print(nmea0183Buffer);
}


/************************* Main Send Function *************************/

// Call in setup() to initialise NMEA output
void setupNmeaOutput()
{
    // Load config from EEPROM at startup
    loadConfigFromEEPROM();
    
    if (NmeaOutputSerial != NULL && NmeaOutputSerial != &Serial) {
        ((HardwareSerial*)NmeaOutputSerial)->begin(baudNMEA);
    }
    
    Serial.println(F("\n*** NMEA Output configured ***"));
    Serial.println(F("Press 'm' to open configuration menu"));
    displayCurrentConfig();
}

// Call in main loop()
void SendNmea0183()
{
    // Check for menu activation
    if (!menuActive) {
        checkMenuActivation();
    } else {
        processMenu();
    }
    
    // [Rest of SendNmea0183 code remains identical...]
    bool isOutputReady = true;
    if (NmeaOutputSerial == &Serial) {
        isOutputReady = Serial.dtr(); 
    }
    
    if (GGA_Available && isOutputReady)
    {
        if (SEND_GPGGA && nmeaGgaTimer >= GPGGA_INTERVAL_MS) {
            BuildGGA0183();
            nmeaGgaTimer = 0;
        }
        
        if (SEND_GPVTG && nmeaVtgTimer >= GPVTG_INTERVAL_MS) {
            BuildVTG0183();
            nmeaVtgTimer = 0;
        }
        
        if (SEND_GPRMC && nmeaRmcTimer >= GPRMC_INTERVAL_MS) {
            BuildRMC0183();
            nmeaRmcTimer = 0;
        }
        
        if (SEND_GPZDA && nmeaZdaTimer >= GPZDA_INTERVAL_MS) {
            BuildZDA0183();
            nmeaZdaTimer = 0;
        }
    }
}
