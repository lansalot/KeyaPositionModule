// Conversion to Hexidecimal
const char *asciiHex = "0123456789ABCDEF";

// the new PANDA sentence buffer
char nmea[100];

// GGA
char fixTime[12];
char latitude[15];
char latNS[3];
char longitude[15];
char lonEW[3];
char fixQuality[2];
char numSats[4];
char HDOP[5];
char altitude[12];
char ageDGPS[10];

// VTG
char vtgHeading[12] = {};
char speedKnots[10] = {};

// GPS heading EMA filter (x10 deg, same scale as BNO yaw) - used by auto-zero WAS
// Fixed alpha: 0.1 (moderate smoothing, adjustable here if needed)
float emaGpsHdg = 0.0f;
static bool emaGpsInit = false;
static const float EMA_GPS_ALPHA = 0.1f;

// RMC (Recommended Minimum specific GNSS data) - Added by FlorianT
char rmcDate[7] = {};   // Date DDMMYY + \0
char rmcMagVar[6] = {}; // Magnetic Variation (up to 999.9)
char rmcMagEW[3] = {};  // E/W indicator

// IMU
char imuHeading[6];
char imuRoll[6];
char imuPitch[6];
char imuYawRate[6];
elapsedMillis badQOStimer;

// ============================================================
// EMA BNO - Anti-jitter filters for yaw, roll, pitch  (FlorianT)
// Alpha = 0.0 => filter DISABLED (raw value)
// Alpha = 0.05~0.30 => filtering active (lower = smoother)
// Adjustable from the serial monitor: commands EY, ER, EP, ES
// ============================================================
float emaYawAlpha = 0.00f;   // Default value yaw
float emaRollAlpha = 0.00f;  // Default value roll
float emaPitchAlpha = 0.00f; // Default value pitch
float emaStopKmh = 1.5f;     // Speed threshold for EMA reset (km/h), 0=permanent filtering

static float emaYaw = 0.0f;
static float emaRoll_f = 0.0f;
static float emaPitch_f = 0.0f;
static bool emaYawInit = false;
static bool emaRollInit = false;
static bool emaPitchInit = false;
// ============================================================

// If odd characters showed up.
void errorHandler()
{
    // nothing at the moment
}

void GGA_Handler() // Rec'd GGA
{
    // fix time
    parser.getArg(0, fixTime);

    // latitude
    parser.getArg(1, latitude);
    parser.getArg(2, latNS);

    // longitude
    parser.getArg(3, longitude);
    parser.getArg(4, lonEW);

    // fix quality
    parser.getArg(5, fixQuality);

    // satellite #
    parser.getArg(6, numSats);

    // HDOP
    parser.getArg(7, HDOP);

    // altitude
    parser.getArg(8, altitude);

    // time of last DGPS update
    parser.getArg(12, ageDGPS);

    GGA_Available = true;

    if (useTM171)
    {
        imuTrigger = true;
        imuTimer = 0;
        BuildNmea();
    }
    else if (useBNO08x)
    {
        imuHandler(); // Get IMU data ready
        BuildNmea();  // Build & send data GPS data to AgIO (Both Dual & Single)
    }
    else if (!useBNO08x)
    {
        itoa(65535, imuHeading, 10); // 65535 is max value to stop AgOpen using IMU in Panda
        BuildNmea();
    }

    gpsReadyTime = systick_millis_count; // Used for GGA timeout (LED's ETC)
}

void readBNO()
{
    if (bno08x.dataAvailable() == true)
    {
        float dqx, dqy, dqz, dqw, dacr;
        uint8_t dac;

        // get quaternion
        bno08x.getQuat(dqx, dqy, dqz, dqw, dacr, dac);
        /*
                    while (bno08x.dataAvailable() == true)
                    {
                        //get quaternion
                        bno08x.getQuat(dqx, dqy, dqz, dqw, dacr, dac);
                    }
        */
        float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
        dqw = dqw / norm;
        dqx = dqx / norm;
        dqy = dqy / norm;
        dqz = dqz / norm;

        float ysqr = dqy * dqy;

        // yaw (z-axis rotation)
        float t3 = +2.0 * (dqw * dqz + dqx * dqy);
        float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
        yaw = atan2(t3, t4);

        // Convert yaw to degrees x10
        yaw = (int16_t)((yaw * -RAD_TO_DEG_X_10));
        if (yaw < 0)
            yaw += 3600;

        // pitch (y-axis rotation)
        float t2 = +2.0 * (dqw * dqy - dqz * dqx);
        t2 = t2 > 1.0 ? 1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;

        // roll (x-axis rotation)
        float t0 = +2.0 * (dqw * dqx + dqy * dqz);
        float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);

        if (steerConfig.IsUseY_Axis)
        {
            roll = asin(t2) * RAD_TO_DEG_X_10;
            pitch = atan2(t0, t1) * RAD_TO_DEG_X_10;
        }
        else
        {
            pitch = asin(t2) * RAD_TO_DEG_X_10;
            roll = atan2(t0, t1) * RAD_TO_DEG_X_10;
        }

        if (invertRoll)
        {
            roll *= -1;
        }

        // -------------------------------------------------------
        // EMA YAW / ROLL / PITCH - reset when vehicle is stationary
        // -------------------------------------------------------
        float speedMs = atof(speedKnots) * 0.5144f; // knots -> m/s
        bool isStationary = (emaStopKmh > 0.0f) && (speedMs < (emaStopKmh / 3.6f));

        if (emaYawAlpha <= 0.0f || isStationary)
        {
            // Filter disabled OR stationary: raw value, EMA catches up
            emaYaw = (float)yaw;
            emaYawInit = false;
        }
        else
        {
            if (!emaYawInit)
            {
                emaYaw = (float)yaw;
                emaYawInit = true;
            }
            else
            {
                float diff = (float)yaw - emaYaw;
                if (diff > 1800.0f)
                    diff -= 3600.0f; // wrap-around (no change needed)
                if (diff < -1800.0f)
                    diff += 3600.0f;
                emaYaw += emaYawAlpha * diff;
                if (emaYaw < 0.0f)
                    emaYaw += 3600.0f;
                if (emaYaw >= 3600.0f)
                    emaYaw -= 3600.0f;
            }
            yaw = (int16_t)emaYaw;
        }

        // -------------------------------------------------------
        // EMA ROLL - reset when stationary
        // -------------------------------------------------------
        if (emaRollAlpha <= 0.0f || isStationary)
        {
            // Filter disabled OR stationary: raw value, EMA catches up
            emaRoll_f = roll;
            emaRollInit = false;
        }
        else
        {
            if (!emaRollInit)
            {
                emaRoll_f = roll;
                emaRollInit = true;
            }
            else
            {
                float diff = roll - emaRoll_f;
                emaRoll_f += emaRollAlpha * diff;
            }
            roll = emaRoll_f;
        }
        // -------------------------------------------------------
        // EMA PITCH - reset when stationary
        // -------------------------------------------------------
        if (emaPitchAlpha <= 0.0f || isStationary)
        {
            emaPitch_f = pitch;
            emaPitchInit = false;
        }
        else
        {
            if (!emaPitchInit)
            {
                emaPitch_f = pitch;
                emaPitchInit = true;
            }
            else
            {
                float diff = pitch - emaPitch_f;
                emaPitch_f += emaPitchAlpha * diff;
            }
            pitch = emaPitch_f;
        }
        // -------------------------------------------------------
    }
}

void imuHandler()
{
    int16_t temp = 0;
    if (useTM171)
    {
        // Fill rest of Panda Sentence - Heading
        itoa(YawV.fValue * 10, imuHeading, 10);

        if (steerConfig.IsUseY_Axis)
        {
            // the pitch x100
            itoa(PitchV.fValue * 10, imuPitch, 10);

            // the roll x100
            itoa(RollV.fValue * 10, imuRoll, 10);
        }
        else
        {
            // the pitch x100
            itoa(RollV.fValue * 10, imuPitch, 10);

            // the roll x100
            itoa(PitchV.fValue * 10, imuRoll, 10);
        }
        itoa(0, imuYawRate, 10);
    }
    else if (useBNO08x)
    {
        // BNO is reading in its own timer
        //  Fill rest of Panda Sentence - Heading
        temp = yaw;
        itoa(temp, imuHeading, 10);

        // the pitch x10
        temp = (int16_t)pitch;
        itoa(temp, imuPitch, 10);

        // the roll x10
        temp = (int16_t)roll;
        itoa(temp, imuRoll, 10);

        // YawRate - 0 for now
        itoa(0, imuYawRate, 10);
    }
}

void BuildNmea(void)
{
    strcpy(nmea, "");

    strcat(nmea, "$PANDA,");

    strcat(nmea, fixTime);
    strcat(nmea, ",");

    strcat(nmea, latitude);
    strcat(nmea, ",");

    strcat(nmea, latNS);
    strcat(nmea, ",");

    strcat(nmea, longitude);
    strcat(nmea, ",");

    strcat(nmea, lonEW);
    strcat(nmea, ",");

    // 6
    strcat(nmea, fixQuality);
    strcat(nmea, ",");

    strcat(nmea, numSats);
    strcat(nmea, ",");

    strcat(nmea, HDOP);
    strcat(nmea, ",");

    strcat(nmea, altitude);
    strcat(nmea, ",");

    // 10
    strcat(nmea, ageDGPS);
    strcat(nmea, ",");

    // 11
    strcat(nmea, speedKnots);
    strcat(nmea, ",");

    // 12
    strcat(nmea, imuHeading);
    strcat(nmea, ",");

    // 13
    strcat(nmea, imuRoll);
    strcat(nmea, ",");

    // 14
    strcat(nmea, imuPitch);
    strcat(nmea, ",");

    // 15
    strcat(nmea, imuYawRate);

    strcat(nmea, "*");

    CalculateChecksum();

    strcat(nmea, "\r\n");

    if (Ethernet_running) // If ethernet running send the GPS there
    {
        int len = strlen(nmea);
        Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
        Eth_udpPAOGI.write(nmea, len);
        Eth_udpPAOGI.endPacket();
    }
}

void CalculateChecksum(void)
{
    int16_t sum = 0;
    int16_t inx = 0;
    char tmp;

    // The checksum calc starts after '$' and ends before '*'
    for (inx = 1; inx < 200; inx++)
    {
        tmp = nmea[inx];

        // * Indicates end of data and start of checksum
        if (tmp == '*')
        {
            break;
        }

        sum ^= tmp; // Build checksum
    }

    byte chk = (sum >> 4);
    char hex[2] = {asciiHex[chk], 0};
    strcat(nmea, hex);

    chk = (sum % 16);
    char hex2[2] = {asciiHex[chk], 0};
    strcat(nmea, hex2);
}

// ============================================================
// Serial commands to adjust EMA BNO filters
//
// EY<value>   => EMA yaw alpha    (e.g. EY0.10)
// ER<value>   => EMA roll alpha   (e.g. ER0.05)
// EP<value>   => EMA pitch alpha  (e.g. EP0.10)
// ES<value>   => speed threshold for EMA reset in km/h (e.g. ES1.0)
// EY0 / ER0 / EP0  => disable the corresponding filter
// ES0          => permanent filtering (no reset when stationary)
// EY?          => display all current values
//
// Called from zAutoZeroMenu.ino (outside active menu)
// ============================================================
bool handleEmaSerialCommand(const String &cmd)
{
    if (cmd.length() < 2)
        return false;

    // Display current state
    if (cmd.startsWith("EY?") || cmd.startsWith("ER?") || cmd.startsWith("EP?") || cmd.startsWith("ES?"))
    {
        Serial.println(F("--- EMA BNO Filters ---"));
        Serial.print(F("  EMA Yaw   alpha : "));
        Serial.println(emaYawAlpha, 3);
        Serial.print(F("  EMA Roll  alpha : "));
        Serial.println(emaRollAlpha, 3);
        Serial.print(F("  EMA Pitch alpha : "));
        Serial.println(emaPitchAlpha, 3);
        Serial.print(F("  Stop threshold  : "));
        Serial.print(emaStopKmh, 1);
        Serial.println(F(" km/h"));
        Serial.println(F("  0.0 = disabled | 0.05~0.30 = active | ES0 = permanent filtering"));
        return true;
    }

    if (cmd.startsWith("EY"))
    {
        float val = cmd.substring(2).toFloat();
        if (val < 0.0f || val > 1.0f)
        {
            Serial.println(F("EY: out of range [0.0-1.0]"));
            return true;
        }
        emaYawAlpha = val;
        emaYawInit = false;
        Serial.print(F("EMA Yaw alpha -> "));
        Serial.println(emaYawAlpha, 3);
        if (emaYawAlpha == 0.0f)
            Serial.println(F("  (YAW filter DISABLED)"));
        return true;
    }

    if (cmd.startsWith("ER"))
    {
        float val = cmd.substring(2).toFloat();
        if (val < 0.0f || val > 1.0f)
        {
            Serial.println(F("ER: out of range [0.0-1.0]"));
            return true;
        }
        emaRollAlpha = val;
        emaRollInit = false;
        Serial.print(F("EMA Roll alpha -> "));
        Serial.println(emaRollAlpha, 3);
        if (emaRollAlpha == 0.0f)
            Serial.println(F("  (ROLL filter DISABLED)"));
        return true;
    }

    if (cmd.startsWith("EP"))
    {
        float val = cmd.substring(2).toFloat();
        if (val < 0.0f || val > 1.0f)
        {
            Serial.println(F("EP: out of range [0.0-1.0]"));
            return true;
        }
        emaPitchAlpha = val;
        emaPitchInit = false;
        Serial.print(F("EMA Pitch alpha -> "));
        Serial.println(emaPitchAlpha, 3);
        if (emaPitchAlpha == 0.0f)
            Serial.println(F("  (PITCH filter DISABLED)"));
        return true;
    }

    if (cmd.startsWith("ES"))
    {
        float val = cmd.substring(2).toFloat();
        if (val < 0.0f || val > 20.0f)
        {
            Serial.println(F("ES: out of range [0.0-20.0 km/h]"));
            return true;
        }
        emaStopKmh = val;
        Serial.print(F("EMA Stop threshold -> "));
        Serial.print(emaStopKmh, 1);
        Serial.println(F(" km/h"));
        if (emaStopKmh == 0.0f)
            Serial.println(F("  (permanent filtering, no reset when stationary)"));
        return true;
    }

    return false; // command not recognized here
}

/*
  $PANDA
  (1) Time of fix

  position
  (2,3) 4807.038,N Latitude 48 deg 07.038' N
  (4,5) 01131.000,E Longitude 11 deg 31.000' E

  (6) 1 Fix quality:
    0 = invalid
    1 = GPS fix(SPS)
    2 = DGPS fix
    3 = PPS fix
    4 = Real Time Kinematic
    5 = Float RTK
    6 = estimated(dead reckoning)(2.3 feature)
    7 = Manual input mode
    8 = Simulation mode
  (7) Number of satellites being tracked
  (8) 0.9 Horizontal dilution of position
  (9) 545.4 Altitude (ALWAYS in Meters, above mean sea level)
  (10) 1.2 time in seconds since last DGPS update
  (11) Speed in knots

  FROM IMU:
  (12) Heading in degrees
  (13) Roll angle in degrees(positive roll = right leaning - right down, left up)

  (14) Pitch angle in degrees(Positive pitch = nose up)
  (15) Yaw Rate in Degrees / second

  CHKSUM
*/

void VTG_Handler()
{
    // vtg heading
    parser.getArg(0, vtgHeading);

    // vtg Speed knots
    parser.getArg(4, speedKnots);

    // Update GPS heading EMA (x10 deg, same scale as BNO yaw)
    float rawHdg = atof(vtgHeading) * 10.0f;
    if (!emaGpsInit)
    {
        emaGpsHdg = rawHdg;
        emaGpsInit = true;
    }
    else
    {
        // Handle 0/3600 wrap-around
        float diff = rawHdg - emaGpsHdg;
        if (diff > 1800.0f)
            diff -= 3600.0f;
        if (diff < -1800.0f)
            diff += 3600.0f;
        emaGpsHdg += EMA_GPS_ALPHA * diff;
        if (emaGpsHdg < 0.0f)
            emaGpsHdg += 3600.0f;
        if (emaGpsHdg >= 3600.0f)
            emaGpsHdg -= 3600.0f;
    }
}

void RMC_Handler() // Added by FlorianT NMEAOUT
{
    // RMC: $GNRMC,093217.90,A,4346.25836945,N,00152.05424878,E,0.053,15.9,121225,1.2,E,D,C*78
    // Index:  0       1 2    3    4    5    6    7     8     9      10
    memset(rmcDate, 0, sizeof(rmcDate));
    memset(rmcMagVar, 0, sizeof(rmcMagVar));
    memset(rmcMagEW, 0, sizeof(rmcMagEW));

    // 8. Date - (DDMMYY)
    parser.getArg(8, rmcDate);

    // 9. Magnetic Variation
    parser.getArg(9, rmcMagVar);

    // 10. Magnetic Variation East/West indicator
    parser.getArg(10, rmcMagEW);
}
