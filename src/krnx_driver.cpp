#include "krnx_driver.hpp"
#include <iostream>
#include <cstring>
#include "krnxlib.h"

static int CONT_NO = 0;
static int as_err_code = 0;

bool connect_to_robot(const std::string& ip) {
    int ret = krnx_Open(CONT_NO, const_cast<char*>(ip.c_str()));
    if (ret < 0) {
        std::cerr << "[Error] Failed to connect to robot at " << ip << "\n";
        return false;
    } else {
        std::cout << "[Info] Successfully connected to robot at " << ip << "\n";
        return true;
    }

void disconnect_robot() {
krnx_Close(CONT_NO);
}

bool send_speed(float f) {
char cmd[64];
snprintf(cmd, sizeof(cmd), "SPEED %.1f", f);
char out[128];
return krnx_ExecMon(CONT_NO, cmd, out, sizeof(out), &as_err_code) >= 0;
}

bool send_move(float x, float y, float z, float o_deg, float a_deg, float t_deg) {
char cmd[256];
snprintf(cmd, sizeof(cmd), "LMOVE TRANS(%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)",
x, y, z, o_deg, a_deg, t_deg);
char out[128];
return krnx_ExecMon(CONT_NO, cmd, out, sizeof(out), &as_err_code) >= 0;
}
