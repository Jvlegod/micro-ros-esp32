#include "ultrasonic.hpp"

bool UltrasonicInit::begin() {
    pinMode(trig_pin_, OUTPUT);
    digitalWrite(trig_pin_, LOW); 
    pinMode(echo_pin_, INPUT);
    return true;
}

void UltrasonicInit::calc() {
        // ensure clean low before trigger
        digitalWrite(trig_pin_, LOW);
        delayMicroseconds(2);

        // 10 µs HIGH pulse on TRIG
        digitalWrite(trig_pin_, HIGH);
        delayMicroseconds(TRIG_PULSE_US);
        digitalWrite(trig_pin_, LOW);

        // measure echo HIGH width (µs)
        unsigned long dur_us = pulseIn(echo_pin_, HIGH, TIMEOUT_US);
        if (dur_us == 0) {
            Serial.println(F("Ultrasonic: timeout"));
            return;
        }

        // distance (cm) = (duration_us * speed_cm_per_us) / 2
        float distance_cm = (dur_us * SOUND_SPEED_CM_PER_US) * 0.5f;

        Serial.print(F("Distance: "));
        Serial.print(distance_cm, 2);
        Serial.println(F(" cm"));
}
