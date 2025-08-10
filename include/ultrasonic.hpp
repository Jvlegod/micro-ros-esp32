#pragma once
#include <Arduino.h>

#define ULTRASONIC_TRIG 27
#define ULTRASONIC_ECHO 21

class UltrasonicInit {
private:
    uint8_t trig_pin_;
    uint8_t echo_pin_;

public:
    // Speed of sound at ~20°C: 0.0343 cm/µs
    static constexpr float SOUND_SPEED_CM_PER_US = 0.0343f;
    static constexpr uint32_t TRIG_PULSE_US = 10;
    static constexpr uint32_t TIMEOUT_US = 30000; // 30 ms (~5 m range)

    explicit UltrasonicInit(uint8_t trigPin = ULTRASONIC_TRIG, uint8_t echoPin = ULTRASONIC_ECHO)
    : trig_pin_(trigPin), echo_pin_(echoPin) {}

    bool begin();

    void calc();
};
