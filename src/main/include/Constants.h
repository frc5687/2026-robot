// Team 5687 2026

#pragma once
#include <frc/geometry/Translation2d.h>
#include <frc/system/plant/DCMotor.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_jerk.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"

namespace Constants {

inline constexpr units::second_t kLoopPeriod = 20_ms;
inline constexpr units::second_t kLogPeriod = 100_ms;
inline constexpr units::volt_t kNominalVoltage = 12_V;
inline constexpr double kVoltageCompensation = 12.0;
inline constexpr double kJoystickDeadband = 0.1;
inline constexpr double kSteerJoystickDeadband = 0.1;

namespace SwerveDrive {
namespace Module {
inline constexpr units::meter_t kWheelRadius = 0.0548_m;
inline constexpr auto kWheelCircumference =
    kWheelRadius * 2.0 * std::numbers::pi_v<double>;
inline constexpr auto kMeterPerTurn = kWheelCircumference / 1_tr;

inline constexpr double kDriveGearRatio =
    (54.0 / 12.0) * (18.0 / 34.0) * (45.0 / 15.0);
// kraken x60 version
inline constexpr double kSteerGearRatio = (48.0 / 18.0) * (96.0 / 16.0);
// x44 version
// inline constexpr double kSteerGearRatio = (42.0 / 12.0) * (96.0 / 16.0);
inline constexpr double kCouplingRatio =
    45.0 / 15.0; // Inverse of the last stage, where we link to the bevel gear
inline constexpr double kFrictionCoefficient = 1.3;

inline constexpr frc::DCMotor kDriveMotor = frc::DCMotor::KrakenX60FOC();

// Esimtaed inertia of the swerve module drive and steer component, these are
// just guesses from other teams
inline constexpr units::kilogram_square_meter_t kDriveInertia = 0.025_kg_sq_m;
inline constexpr units::kilogram_square_meter_t kSteerInertia = 0.004_kg_sq_m;

inline constexpr frc::DCMotor kSteerMotor = frc::DCMotor::KrakenX44FOC();

// v = (ω_motor / reduction) * r
inline constexpr units::meters_per_second_t kMaxModuleLinearSpeed =
    ((kDriveMotor.freeSpeed / kDriveGearRatio) * kWheelRadius) / 1_rad;

inline constexpr units::ampere_t kDriveSlipCurrent =
    120_A; // TODO: Tune, this is the max stator current to prevent sliping of
// the wheels
inline constexpr units::ampere_t kDriveSupplyCurrentLimit = 80_A;
inline constexpr units::ampere_t kSteerSupplyCurrentLimit = 40_A;

inline constexpr bool kDriveInverted =
    false; // true is clockwise positive, false is counter clockwise positive
inline constexpr bool kSteerInverted =
    false; // true is clockwise positive, false is counter clockwise positive

namespace PID {
namespace DriveVelocity {
inline constexpr double kP = 0.3;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
inline constexpr double kS = 0.150;
inline constexpr double kV = 0.112;
inline constexpr double kA = 0.0;
inline constexpr double kSimP = 1.0;
inline constexpr double kSimI = 0.0;
inline constexpr double kSimD = 0.0;
inline constexpr units::volt_t kSimS = 0.0_V;
inline constexpr auto kSimV = 2.35_V / 1_mps;
inline constexpr auto kSimA = 0.0_V / 1_mps_sq;
} // namespace DriveVelocity

namespace SteerPosition {
inline constexpr double kP = 100.0;
inline constexpr double kI = 0.0;
inline constexpr double kD = 1.0;
inline constexpr double kS = 0.0;
inline constexpr double kV = 0.0;
inline constexpr double kA = 0.0;
inline constexpr double kCruiseVelocity = 100.0;
inline constexpr double kAcceleration = 200.0;
inline constexpr double kSimP = 10.0;
inline constexpr double kSimI = 0.0;
inline constexpr double kSimD = 0.0;
inline constexpr units::volt_t kSimS = 1.0_V;
inline constexpr auto kSimV = 0.0_V / 1_rad_per_s;
inline constexpr auto kSimA = 0.0_V / 1_rad_per_s_sq;
} // namespace SteerPosition
} // namespace PID
} // namespace Module

/**
 * Coordinate System
 *
 * <p>(X, Y): X is N or S, N is + Y is W or E, W is +
 *
 * <p>NW (+,+) NE (+,-)
 *
 * <p>SW (-,+) SE (-,-)
 *
 * <p>We go counter-counter clockwise starting at NW of chassis:
 *
 * <p>NW, SW, SE, NE
 *
 * <p>Note: when robot is flipped over, his is clockwise.
 */
inline constexpr int kModuleCount = 4;

inline constexpr units::meter_t kTrackWidth = 0.4826_m;
inline constexpr units::meter_t kWheelBase = 0.635_m;

inline constexpr units::kilogram_t kMass = 45_kg;

inline constexpr std::array<frc::Translation2d, kModuleCount>
    kModuleTranslations{
        frc::Translation2d{+kWheelBase / 2, +kTrackWidth / 2}, // (+, +)
        frc::Translation2d{+kWheelBase / 2, -kTrackWidth / 2}, // (+, -)
        frc::Translation2d{-kWheelBase / 2, +kTrackWidth / 2}, // (-, +)
        frc::Translation2d{-kWheelBase / 2, -kTrackWidth / 2}, // (-, -)
    };

/*
  Rectangular prism I_zz = 1/12 mass (h^2 + w^2)
  h = wheelbase, w = track width
  https://mechanicsmap.psu.edu/websites/centroidtables/centroids3D/centroids3D.html
*/
inline constexpr units::kilogram_square_meter_t kMomentOfInertia =
    units::kilogram_square_meter_t{
        kMass * (kWheelBase * kWheelBase + kTrackWidth * kTrackWidth) / 12};

// Motors cant realistically reach 100% of thier free speed due to, friction,
// load, etc.
inline constexpr double kDriveMotorEfficiency = 0.90;
inline constexpr units::meters_per_second_t kMaxLinearSpeed =
    Module::kMaxModuleLinearSpeed * kDriveMotorEfficiency;
inline constexpr units::radians_per_second_t kMaxAngularSpeed = 10_rad_per_s;

namespace Odometry {
inline constexpr size_t kSignalsPerModule = 5;
inline constexpr size_t kIMUSignals = 2;
inline constexpr size_t kTotalSignals =
    kModuleCount * kSignalsPerModule + kIMUSignals;
} // namespace Odometry

} // namespace SwerveDrive

namespace Flywheel {
inline constexpr double kGearRatio = (24.0 / 18.0);
inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX44FOC(2);

// all lies rn btw
inline constexpr units::inch_t kFlywheelRadius = 4_in;
inline constexpr units::pound_t kFlywheelMass = 1.5_lb;

// Cylinder inertia 1/2 * m * r^2
inline constexpr units::kilogram_square_meter_t kInertia =
    0.5 * kFlywheelMass * kFlywheelRadius * kFlywheelRadius;

inline constexpr bool kLeftInverted = false;
inline constexpr bool kRightInverted = false;

inline constexpr bool kLeftFollowerOpposed = true;
inline constexpr bool kRightFollowerOpposed = true;
inline constexpr bool kRightBottomFollowerOpposed = true;

inline constexpr units::ampere_t kStatorCurrentLimit = 120_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 80_A;
inline constexpr bool kEnableStatorCurrent = true;
inline constexpr bool kEnableSupplyCurrent = true;
namespace PID {
constexpr double kS = 0.20223;   ///< Static friction (V)
constexpr double kV = 0.12044;   ///< Velocity FF (V / motor-RPS)
constexpr double kA = 0.0073372; ///< Acceleration FF (V / motor-RPS²)
constexpr double kP = 0.2;       ///< Proportional (V / motor-RPS error)
constexpr double kI = 0.0;       ///< Integral
constexpr double kD = 0.0;       ///< Derivative
} // namespace PID

// ── SysId Configuration ──────────────────────────────────────────────

constexpr auto kSysIdRampRate = 1_V / 1_s;
constexpr units::volt_t kSysIdStepVoltage{7_V};
constexpr units::second_t kSysIdTimeout{10_s};

// ── Tolerances ───────────────────────────────────────────────────────

constexpr units::revolutions_per_minute_t kAtSetpointTolerance{75_rpm};

// ── Simulation ───────────────────────────────────────────────────────
// WPILib uses rad/s.  Phoenix uses RPS.
//   kV_wpilib = kV_phoenix / (2π)

constexpr double kSimKs = PID::kS;
constexpr double kSimKv = PID::kV / (2.0 * std::numbers::pi);
constexpr double kSimKa = PID::kA / (2.0 * std::numbers::pi);
constexpr double kSimP = PID::kP / (2.0 * std::numbers::pi);
constexpr double kSimD = PID::kD / (2.0 * std::numbers::pi);

constexpr units::radians_per_second_t kSimToleranceRPS{5.0 * 2.0 *
                                                       std::numbers::pi / 60.0};

// ── Filtering ────────────────────────────────────────────────────────

constexpr units::second_t kFilterTime{0.02_s};
constexpr units::second_t kFilterPeriod{0.02_s};

// ── Defaults ─────────────────────────────────────────────────────────

constexpr units::revolutions_per_minute_t kDefaultRPM{4000_rpm};
constexpr units::second_t kAnticipationDuration = 200_ms; // todo figure out
constexpr units::revolutions_per_minute_t kAnticipationBoostRPM{200_rpm};

} // namespace Flywheel

namespace Hood {
inline constexpr int kNumMotors = 1;
inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX44(kNumMotors);
inline constexpr double kGearRatio = 10;

inline constexpr units::moment_of_inertia::kilogram_square_meter_t kMoi =
    1.0_kg_sq_m;
inline constexpr units::meter_t kArmLength = 1.0_m;

inline constexpr units::turn_t kLeftEncoderOffset = 0.365712890625_tr;
inline constexpr units::turn_t kRightEncoderOffset = 0.365712890625_tr;

inline constexpr units::turn_t kMinAngle = 0.0_tr;
inline constexpr units::turn_t kMaxAngle = 1.0_tr;

namespace SimPID {
inline constexpr double kP = 10;
inline constexpr double kI = 0;
inline constexpr double kD = 0;
} // namespace SimPID

namespace PID {
// TODO: Tune
inline constexpr double kP = 0;
inline constexpr double kI = 0;
inline constexpr double kD = 0;
} // namespace PID

} // namespace Hood

namespace Indexer {

namespace Detection {
constexpr units::ampere_t kCurrentDeltaThreshold{
    5_A}; // dI per cycle to trigger
constexpr units::turns_per_second_t kVelocityDipThreshold{
    2_tps};                                           // decel per cycle
constexpr units::ampere_t kSeatedCurrentCeiling{8_A}; // idle current level
constexpr units::second_t kDebounceTime{0.06_s};      // hold detection
} // namespace Detection
inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX44FOC(2);
inline constexpr units::inch_t kRadius = 2_in;
inline constexpr units::pound_t kMass = 1.5_lb;
inline constexpr units::kilogram_square_meter_t kInertia =
    0.5 * kMass * kRadius * kRadius;

inline constexpr bool kLeftInverted = false;
inline constexpr bool kRightInverted = false;
inline constexpr bool kCenterInverted = false;

inline constexpr bool kRightOpposed = false;

inline constexpr units::ampere_t kStatorCurrentLimit = 120_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 20_A;
inline constexpr bool kEnableStatorCurrent = true;
inline constexpr bool kEnableSupplyCurrent = true;

namespace PID {
inline constexpr double kP = 7.0;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
inline constexpr double kV = 0.0;
inline constexpr double kS = 0.0;
inline constexpr double kA = 0.0;
} // namespace PID

inline constexpr double centerkP = 0.0;
inline constexpr double centerkI = 0.0;
inline constexpr double centerkD = 0.0;

inline constexpr double kGearRatio = (42.0 / 9.0);
inline constexpr double kCenterGearRatio = 4;

inline constexpr units::second_t kBallContactDuration = 0.2_s;
inline constexpr double kBallLoadFactor = 0.95;
} // namespace Indexer

namespace Intake {
// Motor config
constexpr bool kInverted = false;
constexpr bool kFollowerOpposed = true;
constexpr units::ampere_t kStatorCurrentLimit = 60.0_A;
constexpr units::ampere_t kSupplyCurrentLimit = 40.0_A;

// Ball detection
constexpr units::ampere_t kBallCurrentThreshold{15_A};

// Sim
inline const frc::DCMotor kMotor = frc::DCMotor::KrakenX60FOC(1);
constexpr units::kilogram_square_meter_t kInertia{0.001};
constexpr double kGearRatio = 1.0;
} // namespace Intake

namespace Geometry {
inline constexpr frc::Transform3d kRobotToTurretLeft{
    frc::Translation3d{units::meter_t{-0.114}, units::meter_t{0.191},
                       units::meter_t{0.372}},
    frc::Rotation3d{0_rad, 0_rad, 0_rad}};

inline constexpr frc::Transform3d kRobotToHoodLeft{
    frc::Translation3d{units::meter_t{-0.228}, units::meter_t{0.191},
                       units::meter_t{0.4344}},
    frc::Rotation3d{0_rad, 0.0_rad, 0_rad}};

inline constexpr frc::Transform3d kRobotToTurretRight{
    frc::Translation3d{units::meter_t{-0.114}, units::meter_t{-0.191},
                       units::meter_t{0.372}},
    frc::Rotation3d{0_rad, 0_rad, 0_rad}};
} // namespace Geometry

namespace Field {
inline const frc::AprilTagFieldLayout kFieldTagLayout =
    frc::AprilTagFieldLayout::LoadField(
        frc::AprilTagField::k2026RebuiltAndyMark);
inline constexpr units::meter_t kFieldLength = 16.540988_m;
inline constexpr units::meter_t kFieldWidth = 8.069326_m;
inline constexpr units::meter_t kCenter = Field::kFieldLength / 2.0;

namespace Hub {
inline constexpr units::meter_t kWidth = 47_in;
inline constexpr units::meter_t kHeight = 72_in; // includes catcher at top
inline constexpr units::meter_t kInnerWidth = 41.7_in;
inline constexpr units::meter_t kInnerHeight = 56.5_in;

inline const frc::Translation3d kTopCenterPoint{
    kFieldTagLayout.GetTagPose(26)->X() + kWidth / 2.0, kFieldWidth / 2.0,
    kHeight};

inline const frc::Translation3d kInnerCenterPoint{
    kFieldTagLayout.GetTagPose(26)->X() + kWidth / 2.0, kFieldWidth / 2.0,
    kInnerHeight};
} // namespace Hub
} // namespace Field

} // namespace Constants
