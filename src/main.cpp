#include <Arduino.h>
#include "ultrasonic.hpp"
#include "mpu6050.hpp"
#include "diff_drive.hpp"

UltrasonicInit ultrasonicSensor(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
MPU6050 mpu(Wire);

DiffDrive::Pins pins{MOTOR0_PWM_PIN0, MOTOR0_PWM_PIN1, MOTOR1_PWM_PIN0, MOTOR1_PWM_PIN1, 
                   ENCODER_PIN_L0, ENCODER_PIN_L1, ENCODER_PIN_R0, ENCODER_PIN_R1};
DiffDrive diff_drive(pins, DIST_PER_PULSE_M, TRACK_WIDTH_M, 50, true);

void setup() {
    Serial.begin(115200);
    diff_drive.begin();
    diff_drive.setPIDGains(100.0f, 45.0f, 1.0f); // Kp, Ki, Kd
    diff_drive.setOutputLimit(255.0f);
    diff_drive.setIntegralLimit(40.0f);
    // if (!ultrasonicSensor.begin()) {
    //     Serial.println("Ultrasonic init failed");
    //     while (1);
    // }

    // if (!MPU6050Init(mpu)) {
    //     Serial.println("MPU6050 init failed");
    //     while (1);
    // }
    // motor_control.motorForward(MotorL, 50);
    // motor_control.motorReverse(MotorR, 255);
    diff_drive.setTargetBodySpeed(0.20f, 0.0f);
}
void loop() {
    delay(1000);
    diff_drive.update();
    Serial.printf("leftCount: %lld, rightCount: %lld\n", diff_drive.leftCount(), diff_drive.rightCount());
    Serial.printf("vLeft: %lf, vRight: %lf\n", diff_drive.vLeft(), diff_drive.vRight());
    Serial.printf("vLinear: %lf, vAngular: %lf\n", diff_drive.vLinear(), diff_drive.vAngular());
}