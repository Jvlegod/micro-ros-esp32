#include <Arduino.h>
#include "ultrasonic.hpp"
#include "mpu6050.hpp"
#include "diff_drive.hpp"
#include "params.hpp"
#include "uros.hpp"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef USE_ULTRASONIC
UltrasonicInit ultrasonicSensor(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
#endif
#ifdef USE_MPU6050
MPU6050 mpu(Wire);
#endif
#ifdef USE_DIFF_DRIVE
DiffDrive::Pins pins{
    MOTOR0_PWM_PIN0, MOTOR0_PWM_PIN1, MOTOR1_PWM_PIN0, MOTOR1_PWM_PIN1, 
    ENCODER_PIN_L0, ENCODER_PIN_L1, ENCODER_PIN_R0, ENCODER_PIN_R1
};
DiffDrive diff_drive(pins, DIST_PER_PULSE_M, TRACK_WIDTH_M, 20, true);
#endif
void setup() {
    Serial.begin(115200);

    uros::InitConfig cfg;
    cfg.ssid = WIFI_SSID;
    cfg.pass = WIFI_PASS;
    cfg.agent_ip = AGENT_IP;
    cfg.agent_port = AGENT_PORT;
    cfg.node_name = "robot_esp32";
    cfg.executor_handles = 2;
    uros::configure(cfg);
    xTaskCreate(uros::uros_task_entry, "uros_handler", 8192, nullptr, 3, nullptr);
#ifdef USE_DIFF_DRIVE
    diff_drive.begin();
    diff_drive.setPIDGains(60.0f, 100.0f, 1.0f);
    diff_drive.setOutputLimit(150.0f);
    diff_drive.setIntegralLimit(40.0f);
#endif

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
#ifdef USE_DIFF_DRIVE
    diff_drive.setTargetBodySpeed(0.0f, 0.0f);
#endif
}

void loop() {
#ifdef USE_DIFF_DRIVE
    diff_drive.update();
#endif

    // const Odom& o = diff_drive.odom();
    // Serial.printf("x=%.3f m  y=%.3f m  yaw=%.3f rad\n", o.x, o.y, o.yaw);
    // Serial.printf("leftCount: %lld, rightCount: %lld\n", diff_drive.leftCount(), diff_drive.rightCount());
    // Serial.printf("vLeft: %lf, vRight: %lf\n", diff_drive.vLeft(), diff_drive.vRight());
    // Serial.printf("vLinear: %lf, vAngular: %lf\n", diff_drive.vLinear(), diff_drive.vAngular());
}
