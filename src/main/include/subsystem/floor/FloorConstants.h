// Team 5687 2026

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>

namespace Constants {

namespace Floor {

inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX60FOC(1);
inline constexpr double kGearRatio = 5.0 / 1.0;
inline constexpr units::kilogram_square_meter_t kInertia{0.001_kg_sq_m};

inline constexpr bool kInverted = false;
inline constexpr bool kEnableFOC = true;
inline constexpr units::ampere_t kStatorCurrentLimit = 120_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 60_A;

namespace PID {
inline constexpr double kP = 0.0;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
inline constexpr double kV = 0.0;
inline constexpr double kS = 0.0;
inline constexpr double kA = 0.0;
} // namespace PID

} // namespace Floor

} // namespace Constants
