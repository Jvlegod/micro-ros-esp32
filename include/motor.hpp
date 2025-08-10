#pragma once

#include <Arduino.h>
#include "ESP32MotorControl.h"

typedef enum : uint8_t {
  MotorL = 0,
  MotorR = 1
} MotorEnum;

#define MOTOR0_PWM_PIN0 22
#define MOTOR0_PWM_PIN1 23

#define MOTOR1_PWM_PIN0 12
#define MOTOR1_PWM_PIN1 13

#define DIST_PER_PULSE_M 0.0001033418636049274