#pragma once

#include <units/time.h>
#include <units/voltage.h>

namespace Constants {

inline constexpr units::second_t kLoopPeriod = 20_ms;
inline constexpr units::second_t kLogPeriod = 100_ms;
inline constexpr units::volt_t kNominalVoltage = 12_V;
inline constexpr double kVoltageCompensation = 12.0;
inline constexpr double kJoystickDeadband = 0.1;
inline constexpr double kSteerJoystickDeadband = 0.1;

} // namespace Constants

