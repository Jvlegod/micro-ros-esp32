#include <Arduino.h>
#include "ultrasonic.hpp"
#include "mpu6050.hpp"
#include "diff_drive.hpp"
#include "params.hpp"
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <WiFi.h>


UltrasonicInit ultrasonicSensor(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
MPU6050 mpu(Wire);

DiffDrive::Pins pins{MOTOR0_PWM_PIN0, MOTOR0_PWM_PIN1, MOTOR1_PWM_PIN0, MOTOR1_PWM_PIN1, 
                   ENCODER_PIN_L0, ENCODER_PIN_L1, ENCODER_PIN_R0, ENCODER_PIN_R1};
DiffDrive diff_drive(pins, DIST_PER_PULSE_M, TRACK_WIDTH_M, 50, true);

void setup() {
    Serial.begin(115200);
    diff_drive.begin();
    diff_drive.setPIDGains(40.0f, 100.0f, 1.5f); // Kp, Ki, Kd
    diff_drive.setOutputLimit(150.0f);
    diff_drive.setIntegralLimit(40.0f);

#ifdef USE_ULTRASONIC
    if (!ultrasonicSensor.begin()) {
        Serial.println("Ultrasonic init failed");
        while (1);
    }
#endif
#ifdef USE_MPU6050
    if (!MPU6050Init(mpu)) {
        Serial.println("MPU6050 init failed");
        while (1);
    }
#endif

    diff_drive.setTargetBodySpeed(0.20f, 0.0f);
}
void loop() {
    diff_drive.update();

    const Odom& o = diff_drive.odom();
    Serial.printf("x=%.3f m  y=%.3f m  yaw=%.3f rad\n", o.x, o.y, o.yaw);
    Serial.printf("leftCount: %lld, rightCount: %lld\n", diff_drive.leftCount(), diff_drive.rightCount());
    Serial.printf("vLeft: %lf, vRight: %lf\n", diff_drive.vLeft(), diff_drive.vRight());
    Serial.printf("vLinear: %lf, vAngular: %lf\n", diff_drive.vLinear(), diff_drive.vAngular());
}