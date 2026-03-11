#ifndef RECEIVER_INPUT_H
#define RECEIVER_INPUT_H

#include <Arduino.h>

//Controller PINS
#define THROTTLE 36
#define YAW 39
#define ROLL 35
#define PITCH 34

struct ReceiverPulseSnapshot {
	unsigned long throttle;
	unsigned long yaw;
	unsigned long roll;
	unsigned long pitch;
};

void setupInputPins();
unsigned long getThrottlePulseWidth();
ReceiverPulseSnapshot getReceiverPulseSnapshot();

#endif // RECEIVER_INPUT_H
