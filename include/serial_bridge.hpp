#pragma once
#include <Arduino.h>

namespace serial_bridge {

#pragma pack(push,1)
struct OdomMsg { float x, y, yaw; float v_linear, v_angular; };
struct CmdVelMsg { float v_x, w_z; };
#pragma pack(pop)

void poll();

bool pop_last_odom(OdomMsg& out);

bool send_cmd_vel(float v_x, float w_z);

} // namespace serial_bridge
