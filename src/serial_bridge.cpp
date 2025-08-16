#include "serial_bridge.hpp"

namespace serial_bridge {

static HardwareSerial* s_port = nullptr;

static OdomMsg g_odom{};
static bool    g_has_odom = false;
static char data[15];
void poll() {
    while (Serial.available() > 0) {
        delay(10);
        uint16_t idx = 0;

        int bytesRead = Serial.readBytes(data, 22);
        if (bytesRead == 22) {
            if (data[0] == 0xAA, data[1] == 0xBB) {
                // Parse the odometry message
                memcpy(&g_odom.x, data + 2, 4);
                memcpy(&g_odom.y, data + 6, 4);
                memcpy(&g_odom.yaw, data + 10, 4);
                memcpy(&g_odom.v_linear, data + 14, 4);
                memcpy(&g_odom.v_angular, data + 18, 4);
                data[bytesRead] = '\0';
            }
        }
        g_has_odom = true;
    }
}

bool pop_last_odom(OdomMsg& out) {
    bool ok = false;

    if (g_has_odom) {
        out = g_odom;
        g_has_odom = false;
        ok = true;
    }

    return ok;
}

bool send_cmd_vel(float v_x, float w_z) {
    uint8_t header[2] = {0xAA, 0xBB};

    CmdVelMsg msg{v_x, w_z};
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&msg);

    Serial.write(header, 2);
    Serial.write(p, sizeof(CmdVelMsg));

    return true;
}


} // namespace serial_bridge
