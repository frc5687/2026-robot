// Team 5687 2026

#include "commands/drive/AlignToEdgeCommand.h"

#include <frc/MathUtil.h>
#include <frc/Timer.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <algorithm>
#include <cmath>
#include <numbers>

#include "Constants.h"
#include "utils/Logger.h"

static constexpr double kMaxTranslationalAccel = 3.0; // m/s²
static constexpr double kMaxAngularAccel = 10.0;       // rad/s²

AlignToEdgeCommand::AlignToEdgeCommand(DriveSubsystem* driveSubsystem,
                                            std::function<double()> throttle,
                                            std::function<double()> strafe,
                                            double constraintFactor,
                                            bool enableSlewRate)
                                            
        : m_driveSubsystem(driveSubsystem),
        m_throttleSupplier(throttle),
        m_strafeSupplier(strafe),
        m_enableSlewRate(enableSlewRate),
        m_driveController(
          Constants::SwerveDrive::PID::Translation::kP,
          0.0,
          0.0,
          frc::TrapezoidProfile<units::meters>::Constraints{
              units::meters_per_second_t{
                  Constants::SwerveDrive::kMaxLinearSpeed.value() *
                  constraintFactor},
              units::meters_per_second_squared_t{kMaxTranslationalAccel *
                                                 constraintFactor}}),
      m_thetaController(
          Constants::SwerveDrive::PID::Rotation::kP,
          0.0,
          0.0,
          frc::TrapezoidProfile<units::radians>::Constraints{
              units::radians_per_second_t{
                  Constants::SwerveDrive::kMaxAngularSpeed.value() *
                  constraintFactor},
              units::radians_per_second_squared_t{kMaxAngularAccel *
                                                  constraintFactor}}) {
    AddRequirements(driveSubsystem);
    SetName("AutoAlignToPose");

    m_thetaController.EnableContinuousInput(
        units::radian_t{-std::numbers::pi},
        units::radian_t{std::numbers::pi});
    m_thetaController.SetTolerance(
        units::radian_t{2.0 * std::numbers::pi / 180.0}); // 2 degrees
    m_driveController.SetTolerance(0.04_m);
}

void AlignToEdgeCommand::Initialize() {
}

void AlignToEdgeCommand::Execute() {

    frc::ChassisSpeeds DriveSubsystem::GetFieldRelativeSpeeds() const {
    auto robotSpeeds = GetChassisSpeeds();
    return frc::ChassisSpeeds::FromRobotRelativeSpeeds(robotSpeeds,
                                                        GetEstimatedHeading());
    }

    frc::Pose2d currentPose = m_driveSubsystem->GetPose();

    frc::Pose2d leftIntake = currentPose.TransformBy(Constants::Geometry::kRobotToIntakeLeft);
    frc::Pose2d rightIntake = currentPose.TransformBy(Constants::Geometry::kRobotToIntakeRight);

    double lowestDistance = 100;
    bool leftCloser;

    for (frc::Pose2d checkedPose: Constants::Field::Zones::kZones) {
            double checkedX1 = (leftIntake.X()-checkedPose.X())/1_m;
            double checkedY1 = (leftIntake.Y()-checkedPose.Y())/1_m;

            bool checkX = std::fabs(checkedX1) < lowestDistance;
            bool checkY = std::fabs(checkedY1) < lowestDistance;

            if (checkY){
            m_targetPose = {currentPose.X(), checkedPose.Y(), currentPose.Rotation()};
            lowestDistance = checkedPose.Y()/1_m;
            leftCloser = true;
            }
            else if (checkX){
            m_targetPose = {checkedPose.X(), currentPose.Y(), currentPose.Rotation()};
            lowestDistance = checkedPose.X()/1_m;
            leftCloser = true;
            }
        }

    for (frc::Pose2d checkedPose: Constants::Field::Zones::kZones) {
            double checkedX2 = (rightIntake.X()-checkedPose.X())/1_m;
            double checkedY2 = (rightIntake.Y()-checkedPose.Y())/1_m;

            bool checkX = std::fabs(checkedX2) < lowestDistance;
            bool checkY = std::fabs(checkedY2) < lowestDistance;

            if (checkY){
            m_targetPose = {currentPose.X(), checkedPose.Y(), currentPose.Rotation()};
            lowestDistance = checkedPose.Y()/1_m;
            leftCloser = false;
            }
            else if (checkX){
            m_targetPose = {checkedPose.X(), currentPose.Y(), currentPose.Rotation()};
            lowestDistance = checkedPose.X()/1_m;
            leftCloser = false;
            }
        }

    double currentDistance =
    currentPose.Translation().Distance(m_targetPose.Translation()).value();

    frc::ChassisSpeeds fieldSpeeds = m_driveSubsystem->GetFieldRelativeSpeeds();

    frc::Translation2d fieldVelocity{
        units::meter_t{fieldSpeeds.vx.value()},
        units::meter_t{fieldSpeeds.vy.value()}};

    frc::Rotation2d toTargetAngle =
    (m_targetPose.Translation() - currentPose.Translation()).Angle();

    frc::Translation2d rotatedVelocity = fieldVelocity.RotateBy(-toTargetAngle);
    double initialVelocity = std::min(0.0, -rotatedVelocity.X().value());

    m_driveController.Reset(units::meter_t{currentDistance},
                            units::meters_per_second_t{initialVelocity});

    m_thetaController.Reset(currentPose.Rotation().Radians(),
                            m_driveSubsystem->GetChassisSpeeds().omega);


    double throttle = m_throttleSupplier();
    double strafe = m_strafeSupplier();

    throttle = frc::ApplyDeadband(throttle, Constants::kJoystickDeadband);
    strafe = frc::ApplyDeadband(strafe, Constants::kJoystickDeadband);

    if (m_enableSlewRate) {
    throttle = m_xLimiter.Calculate(throttle);
    strafe = m_yLimiter.Calculate(strafe);
    }

    m_driveErrorAbs = currentDistance;

    double ffScaler = std::clamp(
        (currentDistance - kFfMinRadius.value()) /
            (kFfMaxRadius - kFfMinRadius).value(),
        0.0, 1.0);

    double driveVelocityScalar =
        m_driveController.GetSetpoint().velocity.value() * ffScaler +
        m_driveController.Calculate(units::meter_t{currentDistance}, 0_m);

    if (currentDistance < m_driveController.GetPositionTolerance()) {
        driveVelocityScalar = 0.0;
    }

    double thetaVelocity =
        m_thetaController.GetSetpoint().velocity.value() * ffScaler +
        m_thetaController.Calculate(currentPose.Rotation().Radians(),
                                    m_targetPose.Rotation().Radians());

    m_thetaErrorAbs = std::abs(
        (currentPose.Rotation() - m_targetPose.Rotation()).Radians().value());

    if (m_thetaErrorAbs < m_thetaController.GetPositionTolerance()) {
        thetaVelocity = 0.0;
    }

    frc::Rotation2d awayFromTargetAngle =
        (currentPose.Translation() - m_targetPose.Translation()).Angle();

    units::meters_per_second_t vx{driveVelocityScalar *
                                   awayFromTargetAngle.Cos()};
    units::meters_per_second_t vy{driveVelocityScalar *
                                   awayFromTargetAngle.Sin()};

    frc::ChassisSpeeds robotSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        vx, strafe, units::radians_per_second_t{thetaVelocity},
        currentPose.Rotation());

    m_driveSubsystem->Drive(robotSpeeds);
}

void AlignToEdgeCommand::End(bool interrupted) {
    m_driveSubsystem->Stop();
}

bool AlignToEdgeCommand::IsFinished() {
    return m_driveController.AtGoal() && m_thetaController.AtGoal();
}
