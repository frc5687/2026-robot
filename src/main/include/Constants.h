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
    80_A; // TODO: Tune, this is the max stator current to prevent sliping of
// the wheels
inline constexpr units::ampere_t kDriveSupplyCurrentLimit = 40_A;
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

namespace Shooting {
inline constexpr units::velocity::meters_per_second_t kMaxSpeedsWhileShooting =
    1.5_mps;
} // namespace Shooting

} // namespace SwerveDrive

namespace Flywheel {

inline constexpr double kGearRatio = (45.0 / 15.0);

// 4 motors total: 1 leader + 3 followers
inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX44FOC(4);

// all lies rn btw
inline constexpr units::inch_t kFlywheelRadius = 3.125_in;
inline constexpr units::pound_t kFlywheelMass = 11_lb;

// Cylinder inertia: 1/2 * m * r^2
inline constexpr units::kilogram_square_meter_t kInertia =
    0.5 * kFlywheelMass * kFlywheelRadius * kFlywheelRadius;

inline constexpr bool kLeaderInverted = false;

inline constexpr bool kFollower1Opposed = false;
inline constexpr bool kFollower2Opposed = true;
inline constexpr bool kFollower3Opposed = true;

inline constexpr units::ampere_t kStatorCurrentLimit = 120_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 80_A;
inline constexpr bool kEnableStatorCurrent = true;
inline constexpr bool kEnableSupplyCurrent = true;

namespace PID {
constexpr double kS = 0.28059;  //< Static friction (V)
constexpr double kV = 0.12273;  //< Velocity FF (V / motor-RPS)
constexpr double kA = 0.008801; //< Acceleration FF (V / motor-RPS²)
constexpr double kP = 0.2;      //< Proportional (V / motor-RPS error)
constexpr double kI = 0.0;      //< Integral
constexpr double kD = 0.0;      //< Derivative
} // namespace PID

inline constexpr units::second_t kClosedLoopRampPeriod = 0.5_s;
constexpr units::revolutions_per_minute_t kAtSetpointTolerance{75_rpm};

constexpr double kSimKs = PID::kS;
constexpr double kSimKv = PID::kV / (2.0 * std::numbers::pi);
constexpr double kSimKa = PID::kA / (2.0 * std::numbers::pi);
constexpr double kSimP = PID::kP / (2.0 * std::numbers::pi);
constexpr double kSimD = PID::kD / (2.0 * std::numbers::pi);

constexpr units::radians_per_second_t kSimToleranceRPS{5.0 * 2.0 *
                                                       std::numbers::pi / 60.0};

constexpr units::second_t kFilterTime{0.02_s};
constexpr units::second_t kFilterPeriod{0.02_s};

constexpr units::revolutions_per_minute_t kDefaultRPM{4000_rpm};
constexpr units::second_t kAnticipationDuration = 200_ms; // todo figure out
constexpr units::revolutions_per_minute_t kAnticipationBoostRPM{200_rpm};

} // namespace Flywheel

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

namespace FloorRoller {

inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX60FOC(2);
inline constexpr units::inch_t kRadius = 2_in;
inline constexpr units::pound_t kMass = 1.5_lb;
inline constexpr units::kilogram_square_meter_t kInertia =
    0.5 * kMass * kRadius * kRadius;

inline constexpr bool kLeaderInverted = false;
inline constexpr bool kFollowerOpposed = true;

inline constexpr units::ampere_t kStatorCurrentLimit = 120_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 20_A;

namespace PID {
inline constexpr double kP = 1.0;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
inline constexpr double kV = 0.0;
inline constexpr double kS = 0.0;
inline constexpr double kA = 0.0;
} // namespace PID

inline constexpr double kGearRatio = (42.0 / 9.0);

} // namespace FloorRoller

namespace Kicker {

inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX60FOC(2);
inline constexpr units::inch_t kRadius = 2_in;
inline constexpr units::pound_t kMass = 1.5_lb;
inline constexpr units::kilogram_square_meter_t kInertia =
    0.5 * kMass * kRadius * kRadius;

inline constexpr bool kLeaderInverted = true;
inline constexpr bool kFollowerOpposed = true;

inline constexpr units::ampere_t kStatorCurrentLimit = 120_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 20_A;

namespace PID {
inline constexpr double kP = 0.6;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
inline constexpr double kV = 0.0;
inline constexpr double kS = 0.0;
inline constexpr double kA = 0.0;
} // namespace PID

inline constexpr double kGearRatio = 24.0 / 22.0;
} // namespace Kicker

// All bogus, TODO: TUNE
namespace IntakeDeployer {
inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX44(1);
inline constexpr double kGearRatio = (30.0 / 11.0) * (2400.0 / 10.0);
inline constexpr units::kilogram_square_meter_t kInertia{0.001_kg_sq_m};

inline constexpr bool kInverted = false;
inline constexpr units::ampere_t kStatorCurrentLimit = 60_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 40_A;

inline constexpr units::meter_t kPinionRadius =
    0.5_in; // TODO: set to actual pinion pitch radius
inline constexpr units::meter_t kPinionCircumference =
    kPinionRadius * 2.0 * std::numbers::pi_v<double>;
inline constexpr double kMetersPerMotorRotation = 0.29 / 7.192;
// kPinionCircumference.value() / kGearRatio;

inline constexpr units::meter_t kRetractedExtension = 0_m;
inline constexpr units::meter_t kMidExtension = 0.18_m;
inline constexpr units::meter_t kDeployedExtension = 0.29_m; // 0.29
inline constexpr units::meter_t kExtensionTolerance = 0.005_m;

inline constexpr units::turn_t kForwardSoftLimit = 10.5_tr; // TODO
inline constexpr units::turn_t kReverseSoftLimit = -0.5_tr; // TODO

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
inline constexpr units::ampere_t kStatorCurrentLimit = 60_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 40_A;

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
inline constexpr units::ampere_t kStatorCurrentLimit = 60_A;
inline constexpr units::ampere_t kSupplyCurrentLimit = 40_A;

namespace PID {
inline constexpr double kP = 0.5;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
inline constexpr double kS = 0.0;
inline constexpr double kV = 0.0;
inline constexpr double kA = 0.0;
} // namespace PID
} // namespace IntakeBottomRoller

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
