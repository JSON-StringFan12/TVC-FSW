#include "Buzzer.h"

// ---------- INTERNAL STATE ----------

enum BuzzerStateEnum {
  BUZZ_IDLE,
  BUZZ_ON,
  BUZZ_OFF
};

static int           s_buzzerPin       = -1;
static BuzzerStateEnum s_state         = BUZZ_IDLE;
static unsigned long s_lastChangeMs    = 0;
static int           s_toneHz          = 0;
static uint8_t       s_beepCount       = 0;   // beeps remaining (including current)
static uint16_t      s_onMs            = 0;
static uint16_t      s_offMs           = 0;

// ---------- PUBLIC API ----------

void Buzzer_init(int pin) {
  s_buzzerPin = pin;
  pinMode(s_buzzerPin, OUTPUT);
  noTone(s_buzzerPin);
  s_state        = BUZZ_IDLE;
  s_beepCount    = 0;
  s_onMs         = 0;
  s_offMs        = 0;
  s_toneHz       = 0;
  s_lastChangeMs = millis();
}

void Buzzer_startPattern(uint8_t count,
                         uint16_t onMs,
                         uint16_t offMs,
                         int toneHz)
{
  if (s_buzzerPin < 0) return;   // not initialized
  if (count == 0) return;

  s_beepCount    = count;
  s_onMs         = onMs;
  s_offMs        = offMs;
  s_toneHz       = toneHz;
  s_lastChangeMs = millis();
  s_state        = BUZZ_ON;

  tone(s_buzzerPin, s_toneHz);
}

void Buzzer_singleBeep(uint16_t onMs,
                       uint16_t offMs,
                       int toneHz)
{
  Buzzer_startPattern(1, onMs, offMs, toneHz);
}

void Buzzer_reportStatus(int status) {
  switch (status) {
    case 0:
      // 3 short beeps = All OK
      Buzzer_startPattern(3, 200, 100, 1500);
      break;
    case 1:
      // 4 long beeps = IMU error
      Buzzer_startPattern(4, 600, 100, 500);
      break;
    case 2:
      // 3 long beeps = SD write error
      Buzzer_startPattern(3, 600, 100, 500);
      break;
    case 3:
      // 2 long beeps = Altimeter error
      Buzzer_startPattern(2, 600, 100, 500);
      break;
    default:
      // no-op
      break;
  }
}

void Buzzer_update() {
  if (s_buzzerPin < 0) return;
  if (s_state == BUZZ_IDLE) return;

  unsigned long now = millis();

  if (s_state == BUZZ_ON) {
    if (now - s_lastChangeMs >= s_onMs) {
      noTone(s_buzzerPin);
      s_lastChangeMs = now;
      s_state        = BUZZ_OFF;
    }
  }
  else if (s_state == BUZZ_OFF) {
    if (now - s_lastChangeMs >= s_offMs) {
      if (s_beepCount > 1) {
        // more beeps to go
        s_beepCount--;
        s_lastChangeMs = now;
        s_state        = BUZZ_ON;
        tone(s_buzzerPin, s_toneHz);
      } else {
        // finished pattern
        s_beepCount = 0;
        s_state     = BUZZ_IDLE;
        noTone(s_buzzerPin);
      }
    }
  }
}
