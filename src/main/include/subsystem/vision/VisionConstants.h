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

inline constexpr frc::Transform3d kRobotToFLCam{
    frc::Translation3d{-0.150868_m, 0.341916_m, 0.293059_m},
    frc::Rotation3d{0_deg, -5_deg, 71_deg}  // roll/pitch/yaw
};

inline constexpr frc::Transform3d kRobotToFRCam{
    frc::Translation3d{-0.150868_m, -0.341916_m, 0.293059_m},
    frc::Rotation3d{0_deg, -5_deg, -71_deg}  // roll/pitch/yaw
};

inline constexpr frc::Transform3d kRobotToBLCam{
    frc::Translation3d{-0.284198_m, 0.274339_m, 0.229020_m},
    frc::Rotation3d{0_deg, -5_deg, 147_deg}};

inline constexpr frc::Transform3d kRobotToBRCam{
    frc::Translation3d{-0.284198_m, -0.274339_m, 0.229020_m},
    frc::Rotation3d{0_deg, -5_deg, -147_deg}};

inline const std::unordered_map<std::string, frc::Transform3d> kTransformMap = {
    {"limelightleft", kRobotToFLCam},
    {"limelight-center", kRobotToBLCam},
    {"limelightleft1", kRobotToFRCam},
    {"limelight-center1", kRobotToBRCam},
    // frc::Rotation3d{147_deg, 5_deg, 0_rad}
};
}  // namespace Constants::Vision
