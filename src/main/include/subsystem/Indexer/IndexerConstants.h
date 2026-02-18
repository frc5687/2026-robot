
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
    inline constexpr bool kLeftMotorInverted = true;
    inline constexpr bool kCenterMotorInverted = true;
    inline constexpr bool kRightMotorInverted = false;


    inline constexpr double kP = 1.0;
    inline constexpr double kI = 0.0;
    inline constexpr double kD = 0.01;

    inline constexpr double centerkP = 0.25;
    inline constexpr double centerkI = 0.0;
    inline constexpr double centerkD = 0.02;

    inline constexpr double kGearRatio = (9.0/42.0);


}  // namespace Constants::Elevator