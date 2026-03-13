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
                                            frc::Pose2d targetPose,
                                            std::string desiredEdge,
                                            std::function<double()> throttle,
                                            std::function<double()> strafe,
                                            double constraintFactor,
                                            bool enableSlewRate)
                                            
        : m_driveSubsystem(driveSubsystem),
        m_targetPose(targetPose),
        m_desiredEdge(desiredEdge),
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
    frc::Pose2d currentPose = m_driveSubsystem->GetPose();

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
}

void AlignToEdgeCommand::Execute() {
    frc::Pose2d currentPose = m_driveSubsystem->GetPose();

    double throttle = m_throttleSupplier();
    double strafe = m_strafeSupplier();

    throttle = ApplyDeadband(throttle, Constants::kJoystickDeadband);
    strafe = ApplyDeadband(strafe, Constants::kJoystickDeadband);

    if (m_enableSlewRate) {
    throttle = m_xLimiter.Calculate(throttle);
    strafe = m_yLimiter.Calculate(strafe);
    }

    double currentDistance =
        currentPose.Translation().Distance(m_targetPose.Translation()).value();
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
