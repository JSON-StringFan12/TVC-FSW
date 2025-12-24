#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// --------- MUST MATCH PRIMARY TEENSY STATE LAYOUT ----------
struct State {
    float yaw;       // Z axis (degrees)
    float pitch;     // Y axis (degrees)
    float roll;      // X axis (degrees)
    float altitude;  // Altitude (m)
    float accel_z;   // Z axis acceleration (m/s^2)
    float accel_y;   // Z axis acceleration (m/s^2)
    float accel_x;   // X axis acceleration (m/s^2)
    float TVCPitch;
    float TVCYaw;
    float time;       // seconds since launch (as defined by primary)
    float launchFlag; // 0.0 = not launched, 1.0 = launched
};

// --------- SIMPLE PACKET PARSER (HEADER + RAW STRUCT) ------
const uint8_t HEADER0 = 0xAA;
const uint8_t HEADER1 = 0x55;

enum ParseState {
  WAIT_HEADER0,
  WAIT_HEADER1,
  READ_PAYLOAD
};

ParseState parseState = WAIT_HEADER0;

State   currentState;
uint8_t payloadBuf[sizeof(State)];
size_t  payloadIndex = 0;

// --------- SD CARD + LOGGING CONTROL -----------------------
const int   SD_CS_PIN               = 15;          // SD CS pin on aux Teensy
const float POST_LAUNCH_LOG_S       = 30.0f;       // high-rate window length
const unsigned long START_LOG_DELAY_MS = 100UL;  // start logging 10s after power-on
const unsigned long SLOW_LOG_PERIOD_MS  = 2000UL;  // 0.5 Hz logging when not in high-rate

File logFile;
bool sdAvailable      = false;
bool headerWritten    = false;
bool loggingStarted   = false;     // logger has waited 10s after power-on
bool highRateActive   = false;     // currently in high-rate window
bool prevLaunched     = false;     // last launchFlag state

unsigned long powerOnMillis       = 0;
unsigned long highRateStartMillis = 0;
unsigned long lastSlowLogMillis   = 0;
unsigned long lastFlushMillis     = 0;

char logFilename[16] = "";         // e.g. "FLIGHT00.CSV"

// --------- SD LOGGING --------------------------------------
void WriteHeaderIfNeeded() {
  if (!sdAvailable || headerWritten || !logFile) return;

  logFile.println(
    "t_launch,launchFlag,yaw,pitch,roll,altitude,accel_x,accel_y,accel_z,TVCPitch,TVCYaw"
  );
  headerWritten = true;
}

void LogStateCSV(const State &s) {
  if (!sdAvailable || !logFile) return;

  WriteHeaderIfNeeded();

  logFile.print(s.time, 6);       logFile.print(',');
  logFile.print(s.launchFlag, 0); logFile.print(',');
  logFile.print(s.yaw, 6);        logFile.print(',');
  logFile.print(s.pitch, 6);      logFile.print(',');
  logFile.print(s.roll, 6);       logFile.print(',');
  logFile.print(s.altitude, 6);   logFile.print(',');
  logFile.print(s.accel_x, 6);    logFile.print(',');
  logFile.print(s.accel_y, 6);    logFile.print(',');
  logFile.print(s.accel_z, 6);    logFile.print(',');
  logFile.print(s.TVCPitch, 6);   logFile.print(',');
  logFile.println(s.TVCYaw, 6);
}

// --------- PARSER UTIL -------------------------------------
void resetParser() {
  parseState   = WAIT_HEADER0;
  payloadIndex = 0;
}

// --------- SETUP ------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // wait up to 4s for USB
  }

  Serial1.begin(115200);  // primary Teensy's Serial1 TX

  Serial.println("Logger Teensy starting...");
  Serial.print("sizeof(State) = ");
  Serial.println(sizeof(State));

  powerOnMillis     = millis();
  lastSlowLogMillis = powerOnMillis;
  lastFlushMillis   = powerOnMillis;

  // SD init
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init FAILED. Logging disabled.");
    sdAvailable = false;
  } else {
    Serial.println("SD init OK.");

    // Find a fresh filename: FLIGHT00.CSV .. FLIGHT99.CSV
    bool fileOpened = false;
    for (int i = 0; i < 100; i++) {
      snprintf(logFilename, sizeof(logFilename), "FLIGHT%02d.CSV", i);
      if (!SD.exists(logFilename)) {
        logFile = SD.open(logFilename, FILE_WRITE);
        if (logFile) {
          fileOpened = true;
          Serial.print("Logging to ");
          Serial.println(logFilename);
        }
        break;
      }
    }

    if (!fileOpened) {
      Serial.println("Failed to create new FLIGHTxx.CSV for writing.");
      sdAvailable = false;
    } else {
      sdAvailable   = true;
      headerWritten = false;
      Serial.println("Will log at 0.5 Hz after 10 s, and high-rate for 30 s after each launchFlag=1.");
    }
  }
}

// --------- LOOP -------------------------------------------
void loop() {
  // Parse incoming bytes from Serial1
  while (Serial1.available() > 0) {
    uint8_t b = Serial1.read();

    switch (parseState) {
      case WAIT_HEADER0:
        if (b == HEADER0) {
          parseState = WAIT_HEADER1;
        }
        break;

      case WAIT_HEADER1:
        if (b == HEADER1) {
          payloadIndex = 0;
          parseState   = READ_PAYLOAD;
        } else if (b == HEADER0) {
          // might be another header start; stay here
        } else {
          resetParser();
        }
        break;

      case READ_PAYLOAD:
        payloadBuf[payloadIndex++] = b;
        if (payloadIndex >= sizeof(State)) {
          // full State struct received
          memcpy(&currentState, payloadBuf, sizeof(State));

          unsigned long now = millis();

          // After 10 s from power-on, we allow logging
          if (!loggingStarted && (now - powerOnMillis >= START_LOG_DELAY_MS)) {
            loggingStarted = true;
            Serial.println("10 s after power-on reached: logging enabled.");
          }

          if (loggingStarted && sdAvailable) {
            bool launched = (currentState.launchFlag >= 0.5f);

            // rising edge of launchFlag: start new high-rate window
            if (!prevLaunched && launched) {
              highRateActive      = true;
              highRateStartMillis = now;
              Serial.println("LaunchFlag rising edge: high-rate window started.");
            }

            if (highRateActive) {
              // high-rate: log every packet
              LogStateCSV(currentState);

              // end of high-rate window
              if (now - highRateStartMillis >= (unsigned long)(POST_LAUNCH_LOG_S * 1000.0f)) {
                highRateActive = false;
                Serial.println("High-rate window ended; back to slow logging.");
              }
            } else {
              // slow logging: 0.5 Hz
              if (now - lastSlowLogMillis >= SLOW_LOG_PERIOD_MS) {
                LogStateCSV(currentState);
                lastSlowLogMillis = now;
              }
            }

            // periodic flush (~1 Hz) so it's safe to power off
            if (now - lastFlushMillis >= 1000UL && logFile) {
              logFile.flush();
              lastFlushMillis = now;
            }

            prevLaunched = launched;
          }

          resetParser();
        }
        break;
    }
  }

  // no delay; event-driven on Serial1 input
}
