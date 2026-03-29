#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation3d.h>
#include <units/length.h>

namespace Constants {

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

namespace Trench {
    inline constexpr frc::Pose2d InsideTopBlue{3_m, 7.5_m, 0_deg};
    inline constexpr frc::Pose2d OutsideTopBlue{6_m, 7.5_m, 180_deg};

    inline constexpr frc::Pose2d InsideBottomBlue{3_m, 0.5_m, 0_deg};
    inline constexpr frc::Pose2d OutsideBottomBlue{6_m, 0.5_m, 180_deg};

    inline constexpr frc::Pose2d InsideTopRed{13_m, 7.5_m, 180_deg};
    inline constexpr frc::Pose2d OutsideTopRed{10_m, 7.5_m, 0_deg};

    inline constexpr frc::Pose2d InsideBottomRed{13_m, 0.5_m, 180_deg};
    inline constexpr frc::Pose2d OutsideBottomRed{10_m, 0.5_m, 0_deg};
} // namespace Trench
} // namespace Field

} // namespace Constants

