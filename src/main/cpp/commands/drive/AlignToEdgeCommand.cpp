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
    SetName("AlignToEdgeCommand");

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

    frc::Pose2d currentPose = m_driveSubsystem->GetPose();

    frc::Pose2d leftIntake = currentPose.TransformBy(Constants::Geometry::kRobotToIntakeLeft);
    frc::Pose2d rightIntake = currentPose.TransformBy(Constants::Geometry::kRobotToIntakeRight);

    Logger::Instance().Log("AlignToEdge/leftIntakePose", leftIntake);
    Logger::Instance().Log("AlignToEdge/rightIntakePose", rightIntake);

    units::meter_t lowestDistance = 1000_m;
    bool useX = true;
    bool faceBlue;
    bool faceDepot;

    frc::Rotation2d angleToFaceWall;

    for (frc::Pose2d checkedPose: Constants::Field::Zones::kZones) {
            units::meter_t xABSDistance = units::math::abs(leftIntake.X()-checkedPose.X());
            units::meter_t yABSDistance = units::math::abs(leftIntake.Y()-checkedPose.Y());
            bool checkX =  xABSDistance < lowestDistance;
            bool checkY =  yABSDistance < lowestDistance;

            if (checkY){
            m_targetPose = {currentPose.X(), checkedPose.Y(), currentPose.Rotation()};
            faceDepot = m_targetPose.Y() < currentPose.Y();
            faceBlue = false;
            Logger::Instance().Log("AlignToEdge/targetPoseBeforeTransform", m_targetPose);
            m_targetPose = m_targetPose.TransformBy(Constants::Geometry::kRobotToIntakeLeft.Inverse());
            lowestDistance = yABSDistance;
            useX = false;
            }
            if (checkX){
            m_targetPose = {checkedPose.X(), currentPose.Y(), currentPose.Rotation()};
            faceBlue = m_targetPose.X() < currentPose.X();
            faceDepot = false;
            Logger::Instance().Log("AlignToEdge/targetPoseBeforeTransform", m_targetPose);
            m_targetPose = m_targetPose.TransformBy(Constants::Geometry::kRobotToIntakeLeft.Inverse());
            lowestDistance = xABSDistance;
            useX = true;
            }
        }

    for (frc::Pose2d checkedPose: Constants::Field::Zones::kZones) {
            units::meter_t xABSDistance = units::math::abs(rightIntake.X()-checkedPose.X());
            units::meter_t yABSDistance = units::math::abs(rightIntake.Y()-checkedPose.Y());
            bool checkX = xABSDistance < lowestDistance;
            bool checkY = yABSDistance < lowestDistance;

            if (checkY){
            m_targetPose = {currentPose.X(), checkedPose.Y(), currentPose.Rotation()};
            faceDepot = m_targetPose.Y() < currentPose.Y();
            faceBlue = false;
            Logger::Instance().Log("AlignToEdge/targetPoseBeforeTransform", m_targetPose);
            m_targetPose = m_targetPose.TransformBy(Constants::Geometry::kRobotToIntakeRight.Inverse());
            lowestDistance = yABSDistance;
            useX = false;
            }
            if (checkX){
            m_targetPose = {checkedPose.X(), currentPose.Y(), currentPose.Rotation()};
            faceBlue = m_targetPose.X() < currentPose.X();
            faceDepot = false;
            Logger::Instance().Log("AlignToEdge/targetPoseBeforeTransform", m_targetPose);
            m_targetPose = m_targetPose.TransformBy(Constants::Geometry::kRobotToIntakeRight.Inverse());
            lowestDistance = xABSDistance;
            useX = true;
            }
        }
    
    double throttle = m_throttleSupplier();
    double strafe = m_strafeSupplier();

    double xInput = frc::ApplyDeadband(throttle, Constants::kJoystickDeadband);
    double yInput = frc::ApplyDeadband(strafe, Constants::kJoystickDeadband);

    bool movingBlue = xInput > 0 && !useX;
    bool movingDepot = yInput > 0 && useX;

    units::degree_t wallOffset = 60_deg;

    if(faceBlue && useX){
        angleToFaceWall = (movingDepot) ? (0_deg + (wallOffset*yInput)) : (0_deg + (wallOffset*yInput));
    }
    else if(!faceBlue && useX){
        angleToFaceWall = (movingDepot) ? (180_deg - wallOffset*yInput) : (180_deg - wallOffset*yInput);
    }
    if(faceDepot && !useX){
        angleToFaceWall = (movingBlue) ? (90_deg - wallOffset*xInput) : (90_deg - wallOffset*xInput);
    }
    else if(!faceDepot && !useX){
        angleToFaceWall = (movingBlue) ? (-90_deg + wallOffset*xInput) : (-90_deg + wallOffset*xInput);
    }

    Logger::Instance().Log("AlignToEdge/angleToFaceWall", angleToFaceWall);
    Logger::Instance().Log("AlignToEdge/yInput", yInput);
    Logger::Instance().Log("AlignToEdge/xInput", xInput);
    Logger::Instance().Log("AlignToEdge/useX", useX);

    m_targetPose = {m_targetPose.X(), m_targetPose.Y(), {angleToFaceWall}};

    double currentDistance =
    currentPose.Translation().Distance(m_targetPose.Translation()).value();

    frc::ChassisSpeeds fieldSpeeds = m_driveSubsystem->GetFieldRelativeSpeeds();

    frc::Translation2d fieldVelocity{
        units::meter_t{fieldSpeeds.vx.value()},
        units::meter_t{fieldSpeeds.vy.value()}};

    frc::Rotation2d awayFromTargetAngle =
        (currentPose.Translation() - m_targetPose.Translation()).Angle();

    if (m_enableSlewRate) {
        xInput = m_xLimiter.Calculate(xInput);
        yInput = m_yLimiter.Calculate(yInput);
    }

    double angleOffFactor = 1 / units::math::abs((currentPose.Rotation().Radians() - m_targetPose.Rotation().Radians())/ 1_rad);

    auto xVelocity = xInput * Constants::SwerveDrive::kMaxLinearSpeed*angleOffFactor;
    auto yVelocity = yInput * Constants::SwerveDrive::kMaxLinearSpeed*angleOffFactor;

    m_driveErrorAbs = currentDistance;

    double ffScaler = std::clamp(
        (currentDistance - kFfMinRadius.value()) /
            (kFfMaxRadius - kFfMinRadius).value(),
        0.0, 1.0);

    m_driveController.SetGoal(units::meter_t{currentDistance});

    double driveVelocityScalar =
        m_driveController.GetSetpoint().velocity.value() * ffScaler +
        m_driveController.Calculate(units::meter_t{currentDistance}, 0_m);

    if (currentDistance < m_driveController.GetPositionTolerance()) {
        driveVelocityScalar = 0.0;
    }

    units::meters_per_second_t vx{driveVelocityScalar *
                                   awayFromTargetAngle.Cos()};
    units::meters_per_second_t vy{driveVelocityScalar *
                                   awayFromTargetAngle.Sin()};

    double thetaVelocity =
        m_thetaController.GetSetpoint().velocity.value() * ffScaler +
        m_thetaController.Calculate(currentPose.Rotation().Radians(),
                                    m_targetPose.Rotation().Radians());

    m_thetaErrorAbs = std::abs(
        (currentPose.Rotation() - m_targetPose.Rotation()).Radians().value());

    if (m_thetaErrorAbs < m_thetaController.GetPositionTolerance()) {
        thetaVelocity = 0.0;
    }

    frc::ChassisSpeeds robotSpeeds;

    if(useX){
        robotSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        vx, -yVelocity, units::radians_per_second_t{thetaVelocity},
        currentPose.Rotation());
    }
    else{
        robotSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        -xVelocity, vy, units::radians_per_second_t{thetaVelocity},
        currentPose.Rotation());
    }
    Logger::Instance().Log("AlignToEdge/movingBlue", movingBlue);
    Logger::Instance().Log("AlignToEdge/faceBlue", faceBlue);
    Logger::Instance().Log("AlignToEdge/faceDepot", faceDepot);
    Logger::Instance().Log("AlignToEdge/movingDepot", movingDepot);
    Logger::Instance().Log("AlignToEdge/currentPose", currentPose);
    Logger::Instance().Log("AlignToEdge/targetPose",m_targetPose);
    Logger::Instance().Log("AlignToEdge/currentDistance", currentDistance);
    Logger::Instance().Log("AlignToEdge/robotSpeeds", robotSpeeds);
    Logger::Instance().Log("AlignToEdge/driveErrorAbs", m_driveErrorAbs);
    Logger::Instance().Log("AlignToEdge/thetaErrorAbs", m_thetaErrorAbs);
    Logger::Instance().Log("AlignToEdge/ffScaler", ffScaler);
    Logger::Instance().Log("AlignToEdge/driveVelocityScalar",
                           driveVelocityScalar);

    m_driveSubsystem->Drive(robotSpeeds);
}

void AlignToEdgeCommand::End(bool interrupted) {
    m_driveSubsystem->Stop();
}

bool AlignToEdgeCommand::IsFinished() {
    return false;
}
