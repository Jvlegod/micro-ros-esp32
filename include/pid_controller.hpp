#pragma once

class PID {
public:

  PID(float kp=0.f, float ki=0.f, float kd=0.f);
  void  setSetpoint(float sp);
  void  setGains(float kp, float ki, float kd);
  void  setOutputLimit(float out_lim);   // |u| <= out_lim
  void  setIntegralLimit(float i_lim);   // |Ki*∫e| <= i_lim
  void  reset(float integral_init=0.f);
  float compute(float measurement, float dt);
private:

  float kp_=0, ki_=0, kd_=0;
  float sp_=0, e_prev_=0, integral_=0;
  bool  inited_=false;
  float out_lim_=0, i_lim_=0; // o symbol no limited
};
