// Team 5687 2026

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>

namespace Constants {

namespace Feeder {

inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX60FOC(2);
inline constexpr units::inch_t kRadius = 2_in;
inline constexpr units::pound_t kMass = 1.5_lb;
inline constexpr units::kilogram_square_meter_t kInertia =
    0.5 * kMass * kRadius * kRadius;

inline constexpr bool kLeaderInverted = true;
inline constexpr bool kEnableFOC = false;
inline constexpr bool kFollowerOpposed = false;

inline constexpr units::ampere_t kStatorCurrentLimit = 800_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 300_A;

namespace VelocityPID {
inline constexpr double kP = 1.0;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
inline constexpr double kV = 0.11571;
inline constexpr double kS = 0.4096;
inline constexpr double kA = 0.0027735;
} // namespace VelocityPID

namespace PositionPID {
inline constexpr double kP = 3.0;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
inline constexpr double kS = 0.0;
inline constexpr double kV = 0.0;
inline constexpr double kA = 0.0;
} // namespace PositionPID

inline constexpr double kGearRatio = (42.0 / 9.0);

// Preclear retract and timeout settings.
inline constexpr units::turn_t kClearanceRetract = 4_tr;
inline constexpr units::turn_t kPositionTolerance = 0.25_tr;
inline constexpr units::second_t kClearanceTimeout = 0.5_s;

} // namespace Feeder

} // namespace Constants
