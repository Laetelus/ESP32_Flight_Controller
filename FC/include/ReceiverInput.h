#ifndef RECEIVER_INPUT_H
#define RECEIVER_INPUT_H

#include <Arduino.h>

//Controller PINS
#define THROTTLE 36
#define YAW 39
#define ROLL 35
#define PITCH 34

struct ReceiverPulseSnapshot {
	int throttle;
	int yaw;
	int roll;
	int pitch;
};

void setupInputPins();
ReceiverPulseSnapshot ReadInput();

#endif // RECEIVER_INPUT_H
