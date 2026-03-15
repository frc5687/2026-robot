#pragma once

#include <numbers>

#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>

namespace Constants {

namespace Hood {
inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX44(1);
inline constexpr double kGearRatio = (48.0 / 14.0) * (200.0 / 10.0);

inline constexpr units::kilogram_square_meter_t kMoi = 1.0_kg_sq_m;
inline constexpr units::meter_t kArmLength = 1.0_m;

inline constexpr units::radian_t kMinAngle = 0.0_rad;
inline constexpr units::radian_t kMaxAngle =
    units::radian_t{2.0 * std::numbers::pi};

inline constexpr bool kInverted = false;
inline constexpr units::ampere_t kStatorCurrentLimit = 60_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 40_A;

inline constexpr bool kEncoderInverted = false;
inline constexpr units::turn_t kEncoderMagnetOffset =
    -0.080078125_tr; // TODO: tune

inline constexpr units::radian_t kRetractedAngle = 0_rad;
inline constexpr units::radian_t kDeployedAngle = 1.0_rad;    // TODO: DO this
inline constexpr units::radian_t kAngleTolerance = 0.035_rad; // ~2 deg

inline constexpr units::turn_t kForwardSoftLimit = 10.5_tr;
inline constexpr units::turn_t kReverseSoftLimit = -0.5_tr;

namespace SimPID {
inline constexpr double kP = 10.0;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
} // namespace SimPID

namespace PID {
inline constexpr double kP = 50.0;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
inline constexpr double kS = 0.0;
inline constexpr double kV = 0.0;
inline constexpr double kA = 0.0;
inline constexpr double kCruiseVelocity = 40.0; // rps
inline constexpr double kAcceleration = 80.0;   // rps/s
} // namespace PID
} // namespace Hood

} // namespace Constants

