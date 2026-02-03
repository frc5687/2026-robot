
#pragma once
#include <units/length.h>
#include <units/mass.h>
#include <units/velocity.h>
#include <units/moment_of_inertia.h>

#include <numbers>

#include "frc/system/plant/DCMotor.h"

#include "units/acceleration.h"
// #include "units/torque.h"

namespace Constants::Indexer {
    inline constexpr bool kLeftMotorInverted = false;
    inline constexpr bool kCenterMotorInverted = false;
    inline constexpr bool kRightMotorInverted = false;

}  // namespace Constants::Elevator