#pragma once

#include "frc/system/plant/DCMotor.h"
#include "units/moment_of_inertia.h"
#include "units/time.h"

namespace Constants::Flywheel {
  // all lies rn btw
  inline constexpr double kGearRatio = 1.0;
  inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX44FOC();

  inline constexpr units::inch_t kFlywheelRadius = 4_in;
  inline constexpr units::pound_t kFlywheelMass = 1.5_lb;

  inline constexpr units::kilogram_square_meter_t kInertia = 0.5 * kFlywheelMass * kFlywheelRadius * kFlywheelRadius;

  inline constexpr bool kMotorInverted = false;

  inline constexpr double kP = 0.01;
  inline constexpr double kI = 0.001;
  inline constexpr double kD = 0.0;

  inline constexpr double kS = 0.0;
  inline constexpr double kV = 0.00162;
  inline constexpr double kA = 0.0;

  inline constexpr double kFilterTime = 0.1;
  inline constexpr units::time::second_t kFilterPeriod = 0.02_s;
}
