// Team 5687 2026

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>

#include <numbers>

namespace Constants {

namespace IntakeDeployer {
inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX44(1);
inline constexpr double kGearRatio = (30.0 / 11.0) * (2400.0 / 10.0);
inline constexpr units::kilogram_square_meter_t kInertia{0.001_kg_sq_m};

inline constexpr bool kInverted = false;
inline constexpr bool kEnableFOC = false;
inline constexpr units::ampere_t kStatorCurrentLimit = 120_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 80_A;

inline constexpr units::ampere_t kDeployedStatorCurrentLimit = 30_A;
inline constexpr units::ampere_t kDeployedSupplyCurrentLimit = 20_A;

inline constexpr units::meter_t kPinionRadius =
    0.5_in; // TODO: set to actual pinion pitch radius
inline constexpr units::meter_t kPinionCircumference =
    kPinionRadius * 2.0 * std::numbers::pi_v<double>;
// kPinionCircumference.value() / kGearRatio;

inline constexpr units::meter_t kRetractedExtension = 0_m;
inline constexpr units::meter_t kMidExtension = 0.185_m;
inline constexpr units::meter_t kDeployedExtension = 0.285_m; // 0.29
inline constexpr units::meter_t kFullyExtend = 0.3_m;
inline constexpr units::turn_t kDeployedMotorRotations = 13.625_tr;
inline constexpr double kMetersPerMotorRotation =
    0.3 / kDeployedMotorRotations.value();

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
inline constexpr double kCruiseVelocity = 70.0;
inline constexpr double kAcceleration = 300.0;
} // namespace PID

namespace SimPID {
inline constexpr double kP = 2.0;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
} // namespace SimPID

namespace Compliance {
inline constexpr units::ampere_t kCurrentThreshold = 30_A;
inline constexpr units::turn_t kPositionErrorThreshold = 0.5_tr;
inline constexpr units::ampere_t kRecoverCurrentThreshold = 5_A;
inline constexpr units::turns_per_second_t kSettledVelocity = 0.5_tps;
inline constexpr units::volt_t kYieldVoltage = 2.0_V;
} // namespace Compliance
} // namespace IntakeDeployer

namespace IntakeTopRoller {
inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX44FOC(2);
inline constexpr double kGearRatio = 2.0 / 1.0;
inline constexpr units::kilogram_square_meter_t kInertia{0.001_kg_sq_m};

inline constexpr bool kLeaderInverted = true;
inline constexpr bool kFollowerOpposed = true;
inline constexpr bool kEnableFOC = false;
inline constexpr units::ampere_t kStatorCurrentLimit = 120_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 15_A;

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

inline constexpr bool kInverted = true;
inline constexpr bool kEnableFOC = true;
inline constexpr units::ampere_t kStatorCurrentLimit = 120_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 10_A;

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
