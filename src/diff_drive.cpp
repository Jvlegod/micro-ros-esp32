#include "diff_drive.hpp"

DiffDrive::DiffDrive(const Pins& pins,
                     float dist_per_pulse_m,
                     float track_width_m,
                     uint32_t sample_ms,
                     bool full_quad)
: pins_(pins),
  dpp_(dist_per_pulse_m),
  track_(track_width_m),
  sample_ms_(sample_ms),
  full_quad_(full_quad) {}

void DiffDrive::begin() {
    // Encoders
    if (full_quad_) {
      encL_.attachFullQuad(pins_.ENC_L_A, pins_.ENC_L_B);
      encR_.attachFullQuad(pins_.ENC_R_A, pins_.ENC_R_B);
    } else {
      encL_.attachHalfQuad(pins_.ENC_L_A, pins_.ENC_L_B);
      encR_.attachHalfQuad(pins_.ENC_R_A, pins_.ENC_R_B);
    }
    prevCntL_ = encL_.getCount();
    prevCntR_ = encR_.getCount();
    tPrev_ = millis();

    // Montor
    motor_ = ESP32MotorControl();
    motor_.attachMotors(pins_.L_IN1, pins_.L_IN2,
                        pins_.R_IN1, pins_.R_IN2);

    // default limited
    pidL_.setOutputLimit(100.f);
    pidR_.setOutputLimit(100.f);
    pidL_.setIntegralLimit(40.f);
    pidR_.setIntegralLimit(40.f);

    stop();
}

void DiffDrive::setPercent(int16_t left_pct, int16_t right_pct) {
    pid_on_ = false;
    writePercents_(left_pct, right_pct);
}

void DiffDrive::setPIDGains(float kp,float ki,float kd){
    pidL_.setGains(kp,ki,kd);
    pidR_.setGains(kp,ki,kd);
}

void DiffDrive::setOutputLimit(float pct){
    pidL_.setOutputLimit(pct);
    pidR_.setOutputLimit(pct);
}

void DiffDrive::setIntegralLimit(float pct){
    pidL_.setIntegralLimit(pct);
    pidR_.setIntegralLimit(pct);
}

void DiffDrive::setTargetBodySpeed(float v, float w){
    setTargetWheelSpeed(v - 0.5f * w * track_, v + 0.5f * w * track_);
}

void DiffDrive::setTargetWheelSpeed(float vL, float vR){
    vL_ref_ = vL; vR_ref_ = vR;
    pidL_.setSetpoint(vL_ref_);
    pidR_.setSetpoint(vR_ref_);
    pid_on_ = true;
}

void DiffDrive::enablePID(bool en){ pid_on_ = en; }


void DiffDrive::stop() {
    motor_.motorForward(MotorL, 0);
    motor_.motorForward(MotorR, 0);
}

void DiffDrive::update() {
    uint32_t now = millis();
    if (now - tPrev_ < sample_ms_) return;

    int64_t cntL = encL_.getCount();
    int64_t cntR = encR_.getCount();
    int64_t dL = cntL - prevCntL_;
    int64_t dR = cntR - prevCntR_;
    float dt = (now - tPrev_) / 1000.0f;

    float vL_raw = ((float)dL * dpp_) / dt; // m/s
    float vR_raw = ((float)dR * dpp_) / dt;

    vL_ = vL_ + alpha_ * (vL_raw - vL_);
    vR_ = vR_ + alpha_ * (vR_raw - vR_);

    v_ = 0.5f * (vL_ + vR_);
    w_ = (vR_ - vL_) / track_;

    // Odom
    float dth  = w_ * dt;
    float th_mid = odom_.yaw + 0.5f * dth;

    odom_.x   += v_ * dt * cosf(th_mid);
    odom_.y   += v_ * dt * sinf(th_mid);
    odom_.yaw  = wrapPi_(odom_.yaw + dth);

    prevCntL_ = cntL;
    prevCntR_ = cntR;
    tPrev_ = now;

    if (pid_on_) {
      float uL = pidL_.compute(vL_, dt);
      float uR = pidR_.compute(vR_, dt);
      // Serial.printf("uL: %lf uR: %lf\n", uL, uR);
      writePercents_(uL, uR);
    }
}

void DiffDrive::writeOnePercent_(MotorEnum side, float pct) {
    float p = constrain(pct, -100.f, 100.f);
    uint8_t duty = (uint8_t)roundf(fabsf(p) * 2.55f); // 0..255
    if (p >= 0) {
      motor_.motorForward(side, duty);
    } else {
      motor_.motorReverse(side, duty);
    }
}
void DiffDrive::writePercents_(float l_pct, float r_pct) {
    writeOnePercent_(MotorL, l_pct + diff_l);
    writeOnePercent_(MotorR, r_pct + diff_r);
}

float DiffDrive::vLeft()   const { return vL_; }

float DiffDrive::vRight()  const { return vR_; }

float DiffDrive::vLinear() const { return v_;  }

float DiffDrive::vAngular()const { return w_;  }

int64_t DiffDrive::leftCount() { return encL_.getCount(); }
int64_t DiffDrive::rightCount() { return encR_.getCount(); }

void DiffDrive::clearCounts() {
    encL_.clearCount();
    encR_.clearCount();
    prevCntL_ = 0;
    prevCntR_ = 0;
}