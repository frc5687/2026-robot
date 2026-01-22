#pragma once

#include "frc/system/plant/DCMotor.h"
#include "units/moment_of_inertia.h"

namespace Constants::Flywheel {
  // im doing a bunch of lies
  inline constexpr double kGearRatio = 5.0;
  inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX44FOC();

  inline constexpr units::kilogram_square_meter_t kInertia = 0.02_kg_sq_m;

  inline constexpr double kP = 0.009;
  inline constexpr double kI = 0.0;
  inline constexpr double kD = 0.0;
}
