#pragma once

#include "frc/system/plant/DCMotor.h"
#include "units/angle.h"
#include "units/moment_of_inertia.h"
namespace Constants::Hood {


    inline constexpr int kNumMotors = 1;
    inline constexpr frc::DCMotor kMotor = frc::DCMotor::Minion(kNumMotors);
    inline constexpr double kGearRatio = 10;

    inline constexpr units::moment_of_inertia::kilogram_square_meter_t kMoi = 0.0_kg_sq_m;
    inline constexpr units::meter_t kArmLength = 0.0_m;
    
    
    inline constexpr units::turn_t kMinAngle = 0.0_tr;
    inline constexpr units::turn_t kMaxAngle = 0.0_tr;

}