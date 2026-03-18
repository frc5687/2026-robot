#pragma once

#include <numbers>

#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>

namespace Constants {

namespace IntakeDeployer {
inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX44(1);
inline constexpr double kGearRatio = (30.0 / 11.0) * (2400.0 / 10.0);
inline constexpr units::kilogram_square_meter_t kInertia{0.001_kg_sq_m};

inline constexpr bool kInverted = false;
inline constexpr bool kEnableFOC = false;
inline constexpr units::ampere_t kStatorCurrentLimit = 120_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 40_A;

inline constexpr units::meter_t kPinionRadius =
    0.5_in; // TODO: set to actual pinion pitch radius
inline constexpr units::meter_t kPinionCircumference =
    kPinionRadius * 2.0 * std::numbers::pi_v<double>;
inline constexpr double kMetersPerMotorRotation = 0.283 / 7.192;
// kPinionCircumference.value() / kGearRatio;

inline constexpr units::meter_t kRetractedExtension = 0_m;
inline constexpr units::meter_t kMidExtension = 0.18_m;
inline constexpr units::meter_t kDeployedExtension = 0.292_m; // 0.29
inline constexpr units::meter_t kExtensionTolerance = 0.005_m;

inline constexpr units::turn_t kForwardSoftLimit = 10.5_tr; // TODO
inline constexpr units::turn_t kReverseSoftLimit = -0.5_tr; // TODO
//
inline constexpr units::second_t kPulseExtendDuration = 0.5_s;
inline constexpr units::second_t kPulseRetractDuration = 0.5_s;
inline constexpr units::second_t kShootDeployerSettleDuration = 0.5_s;

namespace PID {
inline constexpr double kP = 35.0;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
inline constexpr double kS = 0.0;
inline constexpr double kV = 0.0;
inline constexpr double kA = 0.0;
inline constexpr double kCruiseVelocity = 50.0;
inline constexpr double kAcceleration = 100.0;
} // namespace PID

namespace SimPID {
inline constexpr double kP = 2.0;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
} // namespace SimPID
} // namespace IntakeDeployer

namespace IntakeTopRoller {
inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX44FOC(2);
inline constexpr double kGearRatio = 2.0 / 1.0;
inline constexpr units::kilogram_square_meter_t kInertia{0.001_kg_sq_m};

inline constexpr bool kLeaderInverted = true;
inline constexpr bool kFollowerOpposed = true;
inline constexpr bool kEnableFOC = false;
inline constexpr units::ampere_t kStatorCurrentLimit = 120_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 20_A;

namespace PID {
inline constexpr double kP = 0.5;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
inline constexpr double kS = 0.0;
inline constexpr double kV = 0.0;
inline constexpr double kA = 0.0;
} // namespace PID
} // namespace IntakeTopRoller

namespace IntakeBottomRoller {
inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX44FOC(1);
inline constexpr double kGearRatio = 3.0 / 1.0;
inline constexpr units::kilogram_square_meter_t kInertia{0.001_kg_sq_m};

inline constexpr bool kInverted = false;
inline constexpr bool kEnableFOC = false;
inline constexpr units::ampere_t kStatorCurrentLimit = 120_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 20_A;

namespace PID {
inline constexpr double kP = 0.5;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
inline constexpr double kS = 0.0;
inline constexpr double kV = 0.0;
inline constexpr double kA = 0.0;
} // namespace PID
} // namespace IntakeBottomRoller

} // namespace Constants
