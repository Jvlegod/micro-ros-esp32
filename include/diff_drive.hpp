#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>
#include "motor.hpp"
#include "encoder.hpp"
#include "pid_controller.hpp"
#include <math.h>

#define TRACK_WIDTH_M 0.175f

typedef struct Odom {
  float x = 0.0f;
  float y = 0.0f;
  float yaw = 0.0f;
} Odom_t;

class DiffDrive {
public:
  struct Pins {
    // driver pins
    int L_IN1, L_IN2;
    int R_IN1, R_IN2;
    // Encoder pins (A/B)
    int ENC_L_A, ENC_L_B;
    int ENC_R_A, ENC_R_B;
  };

  // dist_per_pulse_m: 每脉冲位移(米/脉冲)
  // track_width_m: 轮距(米)
  DiffDrive(const Pins& pins,
            float dist_per_pulse_m,
            float track_width_m,
            uint32_t sample_ms = 50,
            bool full_quad = false);

  // Odom
  const Odom& odom() { return odom_; }
  void resetOdom(float x=0.f, float y=0.f, float yaw=0.f) { odom_.x=x; odom_.y=yaw==yaw? y: y; odom_.yaw=yaw; }


  void begin();

  // -255..255
  void setPercent(int16_t left_pct, int16_t right_pct);

  void stop();

  // calc speed
  void update();

  // get speed
  float vLeft()   const;   // m/s
  float vRight()  const;   // m/s
  float vLinear() const;   // m/s
  float vAngular()const;   // rad/s

  // read counts
  int64_t leftCount();
  int64_t rightCount();
  
  // clear counts
  void clearCounts();

  void setPIDGains(float kp, float ki, float kd);
  void setOutputLimit(float pct_abs_max);
  void setIntegralLimit(float pct_abs_max);
  void setTargetBodySpeed(float v, float w); // set v(m/s), w(rad/s)
  void setTargetWheelSpeed(float vL, float vR);
  void enablePID(bool en);

private:
  
  Odom_t odom_;

  inline float wrapPi_(float a) const {
    while (a >  PI) a -= 2*PI;
    while (a <= -PI) a += 2*PI;
    return a;
  }

  // opitional
  float diff_l = 0.4;
  float diff_r = 0;

  PID pidL_{6.f, 30.f, 0.2f};
  PID pidR_{6.f, 30.f, 0.2f};
  bool  pid_on_ = false;
  float vL_ref_ = 0.f, vR_ref_ = 0.f;

  void writeOnePercent_(MotorEnum side, float pct);
  void writePercents_(float l_pct, float r_pct);

  Pins pins_;
  ESP32Encoder encL_, encR_;
  ESP32MotorControl motor_;
  float dpp_;              // m/pulse
  float track_;            // m
  uint32_t sample_ms_;
  bool full_quad_;

  // speed status
  float vL_ = 0, vR_ = 0;  // m/s
  float v_  = 0, w_  = 0;  // m/s, rad/s
  float alpha_ = 0.3f;

  // Ticks
  int64_t prevCntL_ = 0, prevCntR_ = 0;
  uint32_t tPrev_ = 0;
};
