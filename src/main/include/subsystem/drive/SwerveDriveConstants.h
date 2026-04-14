#pragma once

#include <array>
#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/system/plant/DCMotor.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <units/voltage.h>

namespace Constants {

namespace SwerveDrive {
namespace Module {
//inline constexpr units::meter_t kWheelRadius = 0.0548_m;
inline constexpr units::meter_t kWheelRadius = 0.05045075_m;
inline constexpr auto kWheelCircumference =
    kWheelRadius * 2.0 * std::numbers::pi_v<double>;
inline constexpr auto kMeterPerTurn = kWheelCircumference / 1_tr;

inline constexpr double kDriveGearRatio =
    (54.0 / 12.0) * (25.0 / 32.0) * (30.0 / 15.0);
// kraken x60 version
inline constexpr double kSteerGearRatio = 287.0 / 11.0;
// x44 version
// inline constexpr double kSteerGearRatio = (42.0 / 12.0) * (96.0 / 16.0);
inline constexpr double kCouplingRatio =
    30.0 / 15.0; // Inverse of the last stage, where we link to the bevel gear
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
inline constexpr double test = kMaxModuleLinearSpeed.value();

inline constexpr units::ampere_t kDriveSlipCurrent =
    80_A; // TODO: Tune, this is the max stator current to prevent sliping of
// the wheels
inline constexpr units::ampere_t kDriveSupplyCurrentLimitAuto = 55_A;
inline constexpr units::ampere_t kSteerSupplyCurrentLimitAuto = 20_A;

inline constexpr units::ampere_t kDriveSupplyCurrentLimitTeleop = 45_A;
inline constexpr units::ampere_t kSteerSupplyCurrentLimitTeleop = 20_A;

inline constexpr units::ampere_t kDriveSupplyCurrentLimitDRS = 50_A;
inline constexpr units::ampere_t kSteerSupplyCurrentLimitDRS = 20_A;

inline constexpr units::ampere_t kDriveSupplyCurrentLimit =
    kDriveSupplyCurrentLimitTeleop;
inline constexpr units::ampere_t kSteerSupplyCurrentLimit =
    kSteerSupplyCurrentLimitTeleop;

inline constexpr bool kDriveInverted =
    false; // true is clockwise positive, false is counter clockwise positive
inline constexpr bool kSteerInverted =
    false; // true is clockwise positive, false is counter clockwise positive
inline constexpr bool kDriveFOC = true;
inline constexpr bool kSteerFOC = false;

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

inline constexpr std::array<units::turn_t, 4> kEncoderOffsets{
    -0.216064453125_tr + 0.5_tr, // FL
    -0.132324421875_tr+ 0.5_tr, // FR
    -0.23681640625_tr + 0.5_tr, // BL
    -0.422119140625_tr + 0.5_tr // BR
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
inline constexpr double kAimkP = 10.0;
inline constexpr double kAimkD = 0.0;
} // namespace Shooting

namespace PID {
namespace Translation {
inline constexpr double kP = 2.5;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
} // namespace Translation
namespace Rotation {
inline constexpr double kP = 10;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
} // namespace Rotation
} // namespace PID
} // namespace SwerveDrive

} // namespace Constants

