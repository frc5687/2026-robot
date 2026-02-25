// Team 5687 2026

#pragma once

#include <frc/geometry/Transform3d.h>
#include <units/angle.h>
#include <units/length.h>

#include <string>
#include <unordered_map>

namespace Constants::Vision {
using units::meter_t;
using units::radian_t;

inline constexpr frc::Transform3d kRobotToCam{
    frc::Translation3d{0.305054_m, 0.0_m, 0.339852_m},
    frc::Rotation3d{0_deg, -30_deg, 0_deg}};

inline const std::unordered_map<std::string, frc::Transform3d> kTransformMap = {
    {"limelight", kRobotToCam},
    // frc::Rotation3d{147_deg, 5_deg, 0_rad}
};
} // namespace Constants::Vision
