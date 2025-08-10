#include "pid_controller.hpp"
#include <cmath>

PID::PID(float kp,float ki,float kd):kp_(kp),ki_(ki),kd_(kd) {}
void PID::setSetpoint(float sp){ sp_=sp; }
void PID::setGains(float kp,float ki,float kd){ kp_=kp; ki_=ki; kd_=kd; }
void PID::setOutputLimit(float x){ out_lim_=std::fabs(x); }
void PID::setIntegralLimit(float x){ i_lim_=std::fabs(x); }
void PID::reset(float i0){ integral_=i0; e_prev_=0; inited_=false; }

float PID::compute(float meas, float dt){
  if(dt<=0) return 0;
  float e = sp_ - meas;
  float de=0;
  if(inited_) de=(e - e_prev_)/dt; else inited_=true;
  e_prev_ = e;

  integral_ += e * dt;
  if(ki_>0 && i_lim_>0){
    float i_term = ki_*integral_;
    if(i_term> i_lim_) integral_= i_lim_/ki_;
    if(i_term<-i_lim_) integral_=-i_lim_/ki_;
  }

  float u = kp_*e + ki_*integral_ + kd_*de;
  if(out_lim_>0){
    if(u> out_lim_) u= out_lim_;
    if(u<-out_lim_) u=-out_lim_;
  }
  return u;
}
