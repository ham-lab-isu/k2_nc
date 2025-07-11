#include <iostream>
#include "gcode_parser.hpp"
#include "krnx_driver.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./gcode_driver <gcode_file>\n";
        return 1;
    }

std::string ip = "192.168.1.2";

if (!connect_to_robot(ip)) return 1;

auto cmds = parse_gcode_file(argv[1]);
for (const auto& cmd : cmds) {
if (cmd.f) send_speed(*cmd.f);

if (cmd.x && cmd.y && cmd.z) {
    float x = *cmd.x;
    float y = *cmd.y;
    float z = *cmd.z;
    float o = cmd.a.value_or(0.0f);  // A → O
    float a = cmd.b.value_or(0.0f);  // B → A
    float t = cmd.c.value_or(0.0f);  // C → T

    send_move(x, y, z, o, a, t);
}

}

disconnect_robot();
return 0;

}
