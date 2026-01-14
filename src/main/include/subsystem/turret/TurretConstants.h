#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <units/moment_of_inertia.h>
#include <frc/system/plant/DCMotor.h>

namespace Constants::Turret {
// THese are all bad guesses right now
inline constexpr bool kMotorInverted = false;
inline constexpr double kGearRatio = 5.0;
inline constexpr units::kilogram_square_meter_t kInertia = 0.04_kg_sq_m;

inline constexpr units::meter_t kTurretRadius = 0.5_m;

inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX44FOC();

inline constexpr double kP = 0.5;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
} /* namespace Constants::Turret  */
