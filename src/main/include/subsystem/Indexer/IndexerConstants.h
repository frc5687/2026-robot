
#pragma once
#include <units/length.h>
#include <units/mass.h>
#include <units/velocity.h>

#include <numbers>

#include "frc/system/plant/DCMotor.h"
#include "units/acceleration.h"

namespace Constants::Indexer {

// set to change
inline constexpr bool kInverted = false;

inline constexpr double kGearRatio = 9.0;

inline constexpr units::meter_t kDrumRadius = 1.125_in;
inline constexpr auto kCircumference =
    2.0 * std::numbers::pi_v<double> * kDrumRadius;

inline constexpr double kRotationsToMeters = 1;

inline constexpr units::kilogram_square_meter_t kInertia = 0.04_kg_sq_m;

inline constexpr units::kilogram_t kMass = 1_kg;

inline constexpr int kNumMotors = 1;
inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX60FOC(kNumMotors);

inline constexpr double kMotorEfficiency = 0.9;

inline constexpr units::meters_per_second_t kMaxVelocity =
    (kMotor.freeSpeed / kGearRatio) * kDrumRadius / 1_rad;

// inline constexpr units::meters_per_second_t kMaxVelocity = 3_mps;
inline constexpr units::meters_per_second_squared_t kMaxAccel = 5_mps_sq;

// inline constexpr units::meters_per_second_squared_t kMaxAccel =
//     (kMotor.stallTorque / kGearRatio * kDrumRadius) /
//     (kMass * kDrumRadius * kDrumRadius);

inline constexpr units::meter_t kPositionTolerance = 0.01_m;

inline constexpr double kP = 3.0;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
}  // namespace Constants::Elevator