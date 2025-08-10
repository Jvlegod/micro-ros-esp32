#include "mpu6050.hpp"

bool MPU6050Init(MPU6050 &mpu) {
    Wire.begin(MPU6050_SDA, MPU6050_SCL);
    byte status = mpu.begin();
    Serial.println(status);
    while(status!=0) return false;
    mpu.calcOffsets(true,true);
    return true;
}