#ifndef RECEIVER_INPUT_H
#define RECEIVER_INPUT_H

#include <Arduino.h>
//Controller PINS
#define THROTTLE 36
#define YAW 39
#define ROLL 35
#define PITCH 34

// ─── Global variables for pulse widths ───────────────────────────
extern volatile unsigned long lastRisingEdgeThrottle;
extern volatile unsigned long throttlePulseWidth;
extern volatile unsigned long lastRisingEdgeYaw;
extern volatile unsigned long yawPulseWidth;
extern volatile unsigned long lastRisingEdgeRoll;
extern volatile unsigned long rollPulseWidth;
extern volatile unsigned long lastRisingEdgePitch;
extern volatile unsigned long pitchPulseWidth;

// ─── Receiver input variables ────────────────────────────────────
extern volatile unsigned long receiver_input_channel_3;
extern volatile unsigned long receiver_input_channel_4;
extern volatile unsigned long receiver_input_channel_1;
extern volatile unsigned long receiver_input_channel_2;

// ─── Mutex locks for thread-safe access ──────────────────────────
extern portMUX_TYPE muxThrottle;
extern portMUX_TYPE muxYaw;
extern portMUX_TYPE muxPitch;
extern portMUX_TYPE muxRoll;

// ─── Interrupt Service Routine declarations ──────────────────────
void IRAM_ATTR handleThrottleInterrupt();
void IRAM_ATTR handleYawInterrupt();
void IRAM_ATTR handleRollInterrupt();
void IRAM_ATTR handlePitchInterrupt();


void setupInputPins();

#endif // RECEIVER_INPUT_H
