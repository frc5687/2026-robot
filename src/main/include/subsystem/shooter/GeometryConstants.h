// Team 5687 2026

#pragma once

#include <frc/geometry/Transform3d.h>
#include <units/length.h>

namespace Constants {

namespace Geometry {
inline constexpr frc::Transform3d kRobotToTurretLeft{
    frc::Translation3d{units::meter_t{-0.114}, units::meter_t{0.191},
                       units::meter_t{0.372}},
    frc::Rotation3d{0_rad, 0_rad, 0_rad}};

inline constexpr frc::Transform3d kRobotToHoodLeft{
    frc::Translation3d{units::meter_t{-0.228}, units::meter_t{0.191},
                       units::meter_t{0.4344}},
    frc::Rotation3d{0_rad, 0.0_rad, 0_rad}};

inline constexpr frc::Transform3d kRobotToTurretRight{
    frc::Translation3d{units::meter_t{-0.114}, units::meter_t{-0.191},
                       units::meter_t{0.372}},
    frc::Rotation3d{0_rad, 0_rad, 0_rad}};
} // namespace Geometry

} // namespace Constants
