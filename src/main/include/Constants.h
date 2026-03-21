#pragma once

#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/geometry/Transform3d.h"
#include <units/time.h>
#include <units/voltage.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

namespace Constants {

inline constexpr units::second_t kLoopPeriod = 20_ms;
inline constexpr units::second_t kLogPeriod = 200_ms;
inline constexpr units::volt_t kNominalVoltage = 12_V;
inline constexpr double kVoltageCompensation = 12.0;
inline constexpr double kJoystickDeadband = 0.1;
inline constexpr double kSteerJoystickDeadband = 0.1;

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

inline constexpr frc::Transform2d kRobotToIntakeLeft{
    frc::Translation2d{units::meter_t{-0.63}, units::meter_t{0.42}},
    frc::Rotation2d{0_rad}};

inline constexpr frc::Transform2d kRobotToIntakeRight{
    frc::Translation2d{units::meter_t{-0.63}, units::meter_t{-0.42}},
    frc::Rotation2d{0_rad}};

} // namespace Geometry

namespace Field {
inline const frc::AprilTagFieldLayout kFieldTagLayout =
    frc::AprilTagFieldLayout::LoadField(
        frc::AprilTagField::k2026RebuiltAndyMark);
inline constexpr units::meter_t kFieldLength = 16.540988_m;
inline constexpr units::meter_t kFieldWidth = 8.069326_m;
inline constexpr units::meter_t kCenter = Field::kFieldLength / 2.0;

namespace Hub {
inline constexpr units::meter_t kWidth = 47_in;
inline constexpr units::meter_t kHeight = 72_in; // includes catcher at top
inline constexpr units::meter_t kInnerWidth = 41.7_in;
inline constexpr units::meter_t kInnerHeight = 56.5_in;

inline const frc::Translation3d kTopCenterPoint{
    kFieldTagLayout.GetTagPose(26)->X() + kWidth / 2.0, kFieldWidth / 2.0,
    kHeight};

inline const frc::Translation3d kInnerCenterPoint{
    kFieldTagLayout.GetTagPose(26)->X() + kWidth / 2.0, kFieldWidth / 2.0,
    kInnerHeight};
} // namespace Hub

namespace Trench{
    inline constexpr frc::Pose2d InsideTopBlue{3_m, 7.5_m, 0_deg};
    inline constexpr frc::Pose2d OutsideTopBlue{6_m, 7.5_m, 0_deg};

    inline constexpr frc::Pose2d InsideBottomBlue{3_m, 0.5_m, 0_deg};
    inline constexpr frc::Pose2d OutsideBottomBlue{6_m, 0.5_m, 0_deg};

    inline constexpr frc::Pose2d InsideTopRed{13_m, 7.5_m, 0_deg};
    inline constexpr frc::Pose2d OutsideTopRed{10_m, 7.5_m, 0_deg};

    inline constexpr frc::Pose2d InsideBottomRed{13_m, 0.5_m, 0_deg};
    inline constexpr frc::Pose2d OutsideBottomRed{10_m, 0.5_m, 0_deg};
} // namespace Field

namespace Zones{
    inline constexpr frc::Pose2d BottomLeftBlue{0_m, 0_m, 0_deg};
    inline constexpr frc::Pose2d TopRightBlue{4_m, 8.07_m, 0_deg};

    inline constexpr frc::Pose2d BottomLeftNeutral{5.2225_m, 0_m, 0_deg};
    inline constexpr frc::Pose2d TopRightNeutral{11.318_m, 8.07_m, 0_deg};

    inline constexpr frc::Pose2d BottomLeftRed{12.563_m, 0_m, 0_deg};
    inline constexpr frc::Pose2d TopRightRed{16.54_m, 8.07_m, 0_deg};

    inline constexpr frc::Pose2d BottomLeftBlueBump{4.0286_m, 1.2659_m, 0_deg};
    inline constexpr frc::Pose2d TopRightBlueBump{5.2225_m, 6.80339_m, 0_deg};

    inline constexpr frc::Pose2d BottomLeftRedBump{11.318_m, 1.2659_m, 0_deg};
    inline constexpr frc::Pose2d TopRightRedBump{12.4968_m, 6.80339_m, 0_deg};

    inline constexpr frc::Pose2d BlueHub{4.6228_m, 4.034663_m, 0_deg};
    inline constexpr frc::Pose2d RedHub{11.915394_m, 4.034663_m, 0_deg};

    inline constexpr std::array<frc::Pose2d,2>kZonesExcludingBumps={BottomLeftBlue, TopRightRed};
    inline constexpr std::array<frc::Pose2d,4>kZonesIncludingBumps={BottomLeftBlueBump, TopRightBlueBump, BottomLeftRedBump, TopRightRedBump};
}
} // namespace Field
} // namespace Constants

