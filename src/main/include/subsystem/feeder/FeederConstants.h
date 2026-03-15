#pragma once

#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>

namespace Constants {

namespace Feeder {

inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX60FOC(4);
inline constexpr units::inch_t kRadius = 2_in;
inline constexpr units::pound_t kMass = 1.5_lb;
inline constexpr units::kilogram_square_meter_t kInertia =
    0.5 * kMass * kRadius * kRadius;

inline constexpr bool kLeaderInverted = false;
inline constexpr bool kFollower1Opposed = true;
inline constexpr bool kFollower2Opposed = true;
inline constexpr bool kFollower3Opposed = false;

inline constexpr units::ampere_t kStatorCurrentLimit = 120_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 20_A;

namespace PID {
inline constexpr double kP = 1.0;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
inline constexpr double kV = 0.0;
inline constexpr double kS = 0.0;
inline constexpr double kA = 0.0;
} // namespace PID

inline constexpr double kGearRatio = (42.0 / 9.0);

} // namespace Feeder

} // namespace Constants

