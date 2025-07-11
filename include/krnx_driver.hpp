#pragma once
#include <string>

bool connect_to_robot(const std::string& ip);
void disconnect_robot();
bool send_speed(float f);
bool send_move(float x, float y, float z, float o_deg, float a_deg, float t_deg);

