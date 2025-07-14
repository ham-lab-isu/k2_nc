#include "gcode_parser.hpp"
#include <fstream>
#include <sstream>
#include <regex>

std::vector<GCodeCommand> parse_gcode_file(const std::string& path) {
    std::ifstream infile(path);
    std::vector<GCodeCommand> commands;
    std::string line;
    std::regex pattern(R"(([XYZFABCL])([-+]?\d*\.?\d+))");

    while (std::getline(infile, line)) {
        if (line.empty() || line[0] == ';' || line[0] == '(') continue;
        if (line.find("G1") == std::string::npos) continue;

        GCodeCommand cmd;
        std::smatch match;
        auto searchStart = line.cbegin();
        
        while (std::regex_search(searchStart, line.cend(), match, pattern)) {
            char code = match[1].str()[0];
            float val = std::stof(match[2]);
            
            switch (code) {
                case 'X': cmd.x = val; break;
                case 'Y': cmd.y = val; break;
                case 'Z': cmd.z = val; break;
                case 'F': cmd.f = val; break;
                case 'A': cmd.a = val; break;
                case 'B': cmd.b = val; break;
                case 'C': cmd.c = val; break;
            }
            searchStart = match.suffix().first;
        }
        commands.push_back(cmd);
    }
    return commands;
}
