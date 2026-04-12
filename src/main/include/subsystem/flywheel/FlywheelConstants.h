#pragma once

#include <numbers>

#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/time.h>
#include <units/velocity.h>

namespace Constants {

namespace Flywheel {

inline constexpr double kGearRatio = (50.0 / 36.0) * (51.0/30.0);

// 4 motors total: 1 leader + 3 followers
inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX60FOC(4);

// all lies rn btw
inline constexpr units::inch_t kFlywheelRadius = 3.125_in;
inline constexpr units::pound_t kFlywheelMass = 11_lb;

inline constexpr units::kilogram_square_meter_t kInertia =
    75.0 * 1.0_lb * 1.0_in * 1.0_in;

inline constexpr bool kLeaderInverted = true;
inline constexpr bool kEnableFOC = true;

inline constexpr bool kFollower1Opposed = false;
inline constexpr bool kFollower2Opposed = true;
inline constexpr bool kFollower3Opposed = true;

inline constexpr units::ampere_t kStatorCurrentLimit = 120_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 25_A;
inline constexpr bool kEnableStatorCurrent = true;
inline constexpr bool kEnableSupplyCurrent = true;

namespace PID {
constexpr double kS = 0.2664;  //< Static friction (V)
constexpr double kV = 0.12;  //< Velocity FF (V / motor-RPS)
constexpr double kA = 0.011385; //< Acceleration FF (V / motor-RPS²)
constexpr double kP = 0.5;      //< Proportional (V / motor-RPS error)
constexpr double kI = 0.05;      //< Integral
constexpr double kD = 0.0;      //< Derivative
} // namespace PID

inline constexpr units::second_t kSpinupRampDuration = 1.0_s;
constexpr units::revolutions_per_minute_t kSpinupRampThreshold{500_rpm};
constexpr units::revolutions_per_minute_t kSpinupRetargetTolerance{10_rpm};
constexpr units::revolutions_per_minute_t kAtSetpointTolerance{75_rpm};

constexpr double kSimKs = PID::kS;
constexpr double kSimKv = PID::kV / (2.0 * std::numbers::pi);
constexpr double kSimKa = PID::kA / (2.0 * std::numbers::pi);
constexpr double kSimP = 3;
constexpr double kSimD = 0;

constexpr units::radians_per_second_t kSimToleranceRPS{5.0 * 2.0 *
                                                       std::numbers::pi / 60.0};

constexpr units::second_t kFilterTime{0.02_s};
constexpr units::second_t kFilterPeriod{0.02_s};

constexpr units::revolutions_per_minute_t kDefaultRPM{4000_rpm};
constexpr units::second_t kAnticipationDuration = 200_ms; // todo figure out
constexpr units::revolutions_per_minute_t kAnticipationBoostRPM{200_rpm};

} // namespace Flywheel

} // namespace Constants
