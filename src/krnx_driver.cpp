#include "krnx_driver.hpp"
#include <iostream>
#include <string>
#include "krnx.h"

// Use C-style API with handle
static int controller_handle = -1;
static const int CONTROLLER_NUMBER = 0;  // Usually 0 or 1
static const int ROBOT_NUMBER = 1;       // Usually 1

bool connect_to_robot(const std::string& ip) {
    // krnx_Open expects (int cont_no, char *hostname)
    // We need to cast away const for the hostname parameter
    char* hostname = const_cast<char*>(ip.c_str());
    controller_handle = krnx_Open(CONTROLLER_NUMBER, hostname);
    
    if (controller_handle < 0) {
        std::cerr << "[Error] Failed to connect to robot at " << ip
                  << " (error code: " << controller_handle << ")\n";
        return false;
    }
    
    std::cout << "[Info] Connected to robot at " << ip
              << " (handle: " << controller_handle << ")\n";
    return true;
}

void disconnect_robot() {
    if (controller_handle >= 0) {
        krnx_Close(controller_handle);
        controller_handle = -1;
        std::cout << "[Info] Disconnected from robot\n";
    }
}

bool send_speed(float f) {
    if (controller_handle < 0) return false;
    
    std::string cmd = "SPEED " + std::to_string(f);
    
    // krnx_Execute expects (int cont_no, int robot_no, const char *program, int exec_num, int step_num, int *as_err_code=NULL)
    int error_code = 0;
    int result = krnx_Execute(controller_handle, ROBOT_NUMBER, cmd.c_str(), 1, 1, &error_code);
    
    if (result != 0) {
        std::cerr << "[Error] SPEED command failed with code: " << result 
                  << ", error code: " << error_code << "\n";
        return false;
    }
    
    return true;
}

bool send_move(float x, float y, float z, float o_deg, float a_deg, float t_deg) {
    if (controller_handle < 0) return false;
    
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "LMOVE TRANS(%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)",
             x, y, z, o_deg, a_deg, t_deg);
    
    // krnx_Execute expects (int cont_no, int robot_no, const char *program, int exec_num, int step_num, int *as_err_code=NULL)
    int error_code = 0;
    int result = krnx_Execute(controller_handle, ROBOT_NUMBER, cmd, 1, 1, &error_code);
    
    if (result != 0) {
        std::cerr << "[Error] LMOVE failed with code: " << result 
                  << ", error code: " << error_code << "\n";
        return false;
    }
    
    return true;
}