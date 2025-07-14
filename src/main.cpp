#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "gcode_parser.hpp"
#include "krnx_driver.hpp"

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    if (argc < 2) {
        std::cerr << "Usage: ./gcode_driver <gcode_file>\n";
        return 1;
    }

    // print the value of all of the arguments
    std::cout << "Arguments:\n";
    for (int i = 0; i < argc; ++i) {
        std::cout << "  argv[" << i << "] = " << argv[i] << "\n";
    }

    std::string ip = "192.168.0.2";
    std::cout << "Connecting to robot at IP: " << ip << "\n";
    if (!connect_to_robot(ip)) {
        rclcpp::shutdown();
        return 1;
    }

    std::cout << "Connected. Press ENTER to begin motion...\n";
    std::cin.get();

    auto cmds = parse_gcode_file(argv[1]);
    std::cout << "Parsed " << cmds.size() << " G-code commands\n";

    for (const auto& cmd : cmds) {
        if (cmd.f) {
            std::cout << "Setting speed to " << *cmd.f << "\n";
            send_speed(*cmd.f);
        }
        
        if (cmd.x && cmd.y && cmd.z) {
            float x = *cmd.x;
            float y = *cmd.y;
            float z = *cmd.z;
            float o = cmd.a.value_or(0.0f);  // A → O
            float a = cmd.b.value_or(0.0f);  // B → A
            float t = cmd.c.value_or(0.0f);  // C → T
            
            std::cout << "Moving to X=" << x << " Y=" << y << " Z=" << z 
                      << " O=" << o << " A=" << a << " T=" << t << "\n";
            send_move(x, y, z, o, a, t);
        }
    }

    disconnect_robot();
    rclcpp::shutdown();
    return 0;
}
