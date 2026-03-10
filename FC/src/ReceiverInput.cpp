#include "ReceiverInput.h"
#include "Flight_Controller.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

// ─── Global variables for pulse widths ───────────────────────────
volatile unsigned long lastRisingEdgeThrottle, throttlePulseWidth;
volatile unsigned long lastRisingEdgeYaw, yawPulseWidth;
volatile unsigned long lastRisingEdgeRoll, rollPulseWidth;
volatile unsigned long lastRisingEdgePitch, pitchPulseWidth;


// ─── Mutex locks for thread-safe access ──────────────────────────
portMUX_TYPE muxThrottle = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxYaw = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxPitch = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxRoll = portMUX_INITIALIZER_UNLOCKED;

// ─── Interrupt Service Routines for each channel ──────────────────
void IRAM_ATTR handleThrottleInterrupt()
{
  portENTER_CRITICAL_ISR(&muxThrottle);
  // GPIO port manipulation starting pg 49 in the esp32 tech manual
  if (GPIO.in1.val & (1ULL << (THROTTLE - 32)))
  {
    lastRisingEdgeThrottle = esp_timer_get_time();
  }
  else
  {
    throttlePulseWidth = esp_timer_get_time() - lastRisingEdgeThrottle;
  }
  portEXIT_CRITICAL_ISR(&muxThrottle);
}

void IRAM_ATTR handleYawInterrupt()
{
  portENTER_CRITICAL_ISR(&muxYaw);
  if (GPIO.in1.val & (1ULL << (YAW - 32)))
  {
    lastRisingEdgeYaw = esp_timer_get_time();
  }
  else
  {
    yawPulseWidth = esp_timer_get_time() - lastRisingEdgeYaw;
  }
  portEXIT_CRITICAL_ISR(&muxYaw);
}

void IRAM_ATTR handleRollInterrupt()
{
  portENTER_CRITICAL_ISR(&muxRoll);
  if (GPIO.in1.val & (1ULL << (ROLL - 32)))
  {
    lastRisingEdgeRoll = esp_timer_get_time();
  }
  else
  {
    rollPulseWidth = esp_timer_get_time() - lastRisingEdgeRoll;
  }
  portEXIT_CRITICAL_ISR(&muxRoll);
}

void IRAM_ATTR handlePitchInterrupt()
{
  portENTER_CRITICAL_ISR(&muxPitch);
  if (GPIO.in1.val & (1ULL << (PITCH - 32)))
  {
    lastRisingEdgePitch = esp_timer_get_time();
  }
  else
  {
    pitchPulseWidth = esp_timer_get_time() - lastRisingEdgePitch;
  }
  portEXIT_CRITICAL_ISR(&muxPitch);
}

void setupInputPins()
{
  pinMode(THROTTLE, INPUT);
  pinMode(YAW, INPUT);
  pinMode(PITCH, INPUT);
  pinMode(ROLL, INPUT);

  attachInterrupt(digitalPinToInterrupt(THROTTLE), handleThrottleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(YAW), handleYawInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROLL), handleRollInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PITCH), handlePitchInterrupt, CHANGE);
}
