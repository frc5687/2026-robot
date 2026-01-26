#pragma once

#include "frc/system/plant/DCMotor.h"
#include "units/moment_of_inertia.h"

namespace Constants::Flywheel {
  // im doing a bunch of lies
  inline constexpr double kGearRatio = 1.0;
  inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX44FOC();

  inline constexpr units::inch_t kRadius = 4_in;
  inline constexpr units::pound_t kMass = 1.5_lb;

  inline constexpr units::kilogram_square_meter_t kInertia = 0.5 * kMass * kRadius * kRadius;

  inline constexpr double kP = 0.0;
  inline constexpr double kI = 0.0;
  inline constexpr double kD = 0.0;

  inline constexpr double kS = 0.0001;
  inline constexpr double kV = 0.001400;
  inline constexpr double kA = 0.0003;
}
