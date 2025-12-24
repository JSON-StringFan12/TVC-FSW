#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

// Simple non-blocking buzzer driver using millis()

// Call this once in setup()
void Buzzer_init(int pin);

// Call this every loop()
void Buzzer_update();

// Generic pattern: N beeps, each with onMs / offMs, at toneHz
void Buzzer_startPattern(uint8_t count,
                         uint16_t onMs,
                         uint16_t offMs,
                         int toneHz);

// Convenience: single beep (kept similar to your old ActivateBuzzer)
void Buzzer_singleBeep(uint16_t onMs,
                       uint16_t offMs,
                       int toneHz);

// Status patterns (copies your old semantics):
//  0 -> 3 short beeps = All OK
//  1 -> 4 long beeps  = IMU error
//  2 -> 3 long beeps  = SD write error
//  3 -> 2 long beeps  = Altimeter error
void Buzzer_reportStatus(int status);

#endif // BUZZER_H
