#pragma once
#include <string>
#include <vector>
#include <optional>

struct GCodeCommand {
std::optional<float> x, y, z, f; // Position + feedrate
std::optional<float> a, b, c; // Orientation: A=B, B=A, C=T
};

std::vector<GCodeCommand> parse_gcode_file(const std::string& path);
