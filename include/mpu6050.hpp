#pragma once
#include <Arduino.h>
#include "Wire.h"
#include <MPU6050_light.h>


#define MPU6050_SDA 18
#define MPU6050_SCL 19

bool MPU6050Init(MPU6050 &mpu);