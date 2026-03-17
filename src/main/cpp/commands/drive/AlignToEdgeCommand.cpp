// Team 5687 2026
#include <iostream>
#include "commands/drive/AlignToEdgeCommand.h"

#include <frc/MathUtil.h>
#include <frc/Timer.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include "frc/DriverStation.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <ostream>
#include <vector>

#include "Constants.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Transform2d.h"
#include "units/angle.h"
#include "utils/Logger.h"

static constexpr double kMaxTranslationalAccel = 2.0; // m/s²
static constexpr double kMaxAngularAccel = 20.0;       // rad/s²


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
        frc::Pose2d currentPose = m_driveSubsystem->GetPose();


    frc::ChassisSpeeds fieldSpeeds = m_driveSubsystem->GetFieldRelativeSpeeds();

    frc::Translation2d fieldVelocity{
        units::meter_t{fieldSpeeds.vx.value()},
        units::meter_t{fieldSpeeds.vy.value()}};

    m_thetaController.Reset(currentPose.Rotation().Radians(),
                            m_driveSubsystem->GetChassisSpeeds().omega);
}

bool IsOnField(frc::Pose2d intake){
    bool isOnFieldX = (intake.X() > Constants::Field::Zones::BottomLeftBlue.X()) && (intake.X() < Constants::Field::Zones::TopRightRed.X());
    bool isOnFieldY = (intake.Y() > Constants::Field::Zones::BottomLeftBlue.Y()) && (intake.Y() < Constants::Field::Zones::TopRightRed.Y());
    bool isValid = isOnFieldX && isOnFieldY;
    return isValid;
}

struct EdgeParams {
    frc::Pose2d targetPose;
    bool useXAxis;
    units::meter_t minDistance;
    units::meter_t xMinDistance;
    units::meter_t yMinDistance;
    frc::Pose2d foundEdge;
};

std::span<const frc::Pose2d> GetZones(bool isValidForBump) {
    return isValidForBump ? std::span<const frc::Pose2d>{Constants::Field::Zones::kZonesIncludingBumps}
                    : std::span<const frc::Pose2d>{Constants::Field::Zones::kZonesExcludingBumps};
}

EdgeParams CalculateEdge(const frc::Pose2d& intakePose, bool isValidForBump, const frc::Transform2d& intakeTransform){
    frc::Pose2d targetPose;
    frc::Pose2d foundEdge;
    bool useXAxis;
    
        auto Zones = GetZones(isValidForBump);

        std::vector<frc::Pose2d> loggedZones(Zones.begin(), Zones.end());
        Logger::Instance().Log("AlignToEdge/Zones", loggedZones);

        units::meter_t lowestDistance = 1000_m;

        units::meter_t XLowestDistance;
        units::meter_t YLowestDistance;

        for (frc::Pose2d checkedPose: Zones) {
            units::meter_t xABSDistance = units::math::abs(intakePose.X()-checkedPose.X());
            units::meter_t yABSDistance = units::math::abs(intakePose.Y()-checkedPose.Y());

            bool checkX =  xABSDistance < lowestDistance;

            if (checkX){
            targetPose = {checkedPose.X(), intakePose.Y(), intakePose.Rotation()};
            lowestDistance = xABSDistance;
            useXAxis = true;
            }

            bool checkY =  yABSDistance < lowestDistance && !isValidForBump;

            XLowestDistance = xABSDistance;
            YLowestDistance = yABSDistance;

            if (checkY){
            targetPose = {intakePose.X(), checkedPose.Y(), intakePose.Rotation()};
            lowestDistance = yABSDistance;
            useXAxis = false;
            }

        }
        foundEdge = targetPose; 
        targetPose = targetPose.TransformBy(intakeTransform.Inverse());

    return {
        .targetPose = targetPose,
        .useXAxis = useXAxis,
        .minDistance = lowestDistance,
        .xMinDistance = XLowestDistance,
        .yMinDistance = YLowestDistance,
        .foundEdge = foundEdge
    };
}

int EdgeDirection(frc::Pose2d currentPose, frc::Pose2d foundEdge, bool useXAxis){
    int direction;

    if(useXAxis){
        if(currentPose.X() < foundEdge.X()){
            direction = 1;
            //east
        }
        else {
            direction = 2;
            //west
        }
    }
    else{
        if(currentPose.Y() < foundEdge.Y()){
            direction = 3;
            //north
        }
        else {
            direction = 4;
            //south
        }
    }

    return direction;
}

void AlignToEdgeCommand::Execute() {

    frc::Pose2d currentPose = m_driveSubsystem->GetPose();

    frc::Pose2d leftIntake = currentPose.TransformBy(Constants::Geometry::kRobotToIntakeLeft);
    frc::Pose2d rightIntake = currentPose.TransformBy(Constants::Geometry::kRobotToIntakeRight);

    Logger::Instance().Log("AlignToEdge/leftIntakePose", leftIntake);
    Logger::Instance().Log("AlignToEdge/rightIntakePose", rightIntake);

    frc::Rotation2d angleToFaceWall;

    double maxHubSnap = 3.5;

    double leftIntakeDistanceFromRedHub = leftIntake.Translation().Distance(Constants::Field::Zones::RedHub.Translation()).value();
    double rightIntakeDistanceFromRedHub = rightIntake.Translation().Distance(Constants::Field::Zones::RedHub.Translation()).value();
    bool isValidForRedBump = (leftIntakeDistanceFromRedHub < maxHubSnap) && (rightIntakeDistanceFromRedHub < maxHubSnap);

    double leftIntakeDistanceFromBlueHub = leftIntake.Translation().Distance(Constants::Field::Zones::BlueHub.Translation()).value();
    double rightIntakeDistanceFromBlueHub = rightIntake.Translation().Distance(Constants::Field::Zones::BlueHub.Translation()).value();
    bool isValidForBlueBump = (leftIntakeDistanceFromBlueHub < maxHubSnap) && (rightIntakeDistanceFromBlueHub < maxHubSnap);

    bool isValidForBump = isValidForRedBump || isValidForBlueBump;

    bool poseOnField = (IsOnField(rightIntake)) && (IsOnField(leftIntake));

    auto EdgeLeftIntake = CalculateEdge(leftIntake, isValidForBump, Constants::Geometry::kRobotToIntakeLeft);
    auto EdgeRightIntake = CalculateEdge(rightIntake, isValidForBump, Constants::Geometry::kRobotToIntakeRight);
    auto Edge = (EdgeLeftIntake.minDistance < EdgeRightIntake.minDistance) ? EdgeLeftIntake : EdgeRightIntake;


    double throttle = m_throttleSupplier();
    double strafe = m_strafeSupplier();

    double xInput = frc::ApplyDeadband(throttle, Constants::kJoystickDeadband);
    double yInput = frc::ApplyDeadband(strafe, Constants::kJoystickDeadband);

    int direction = EdgeDirection(currentPose, Edge.foundEdge, Edge.useXAxis);

    units::angle::degree_t angleOffset = 70_deg;
    switch(direction){
        case 1: //east
            angleToFaceWall = 180_deg - yInput*angleOffset;
            break;
        
        case 2: //west
            angleToFaceWall = 0_deg + yInput*angleOffset;
            break;

        case 3: //north
            angleToFaceWall = 270_deg + xInput*angleOffset;
            break;

        case 4: //south
            angleToFaceWall = 90_deg - xInput*angleOffset;
            break;
    }

    Edge.targetPose = {Edge.targetPose.X(), Edge.targetPose.Y(), angleToFaceWall};

    units::angle::degree_t normalizedAngle = (currentPose.Rotation().Degrees().value() < 0) ? currentPose.Rotation().Degrees() + 360_deg : currentPose.Rotation().Degrees();
    units::angle::degree_t normalizedDesiredAngle = (Edge.targetPose.Rotation().Degrees().value() < 0) ? Edge.targetPose.Rotation().Degrees() + 360_deg : Edge.targetPose.Rotation().Degrees();

    units::angle::degree_t angleTolerance = 30_deg;
    units::angle::degree_t angleDifference = currentPose.Rotation().RelativeTo(Edge.targetPose.Rotation()).Degrees();
    bool isAngleValid = angleTolerance > units::math::abs(angleDifference);

    double currentDistance =
    currentPose.Translation().Distance(Edge.targetPose.Translation()).value();

    frc::ChassisSpeeds fieldSpeeds = m_driveSubsystem->GetFieldRelativeSpeeds();

    frc::Translation2d fieldVelocity{
        units::meter_t{fieldSpeeds.vx.value()},
        units::meter_t{fieldSpeeds.vy.value()}};

    frc::Rotation2d awayFromTargetAngle =
        (currentPose.Translation() - Edge.targetPose.Translation()).Angle();

    double flipInput = 1;

    std::optional<frc::DriverStation::Alliance> alliance =
      frc::DriverStation::GetAlliance();

    if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed) {
        flipInput = -1;
    }

    if (m_enableSlewRate) {
        xInput = flipInput*m_xLimiter.Calculate(xInput);
        yInput = flipInput*m_yLimiter.Calculate(yInput);
    }

    double inputVelocityLimit = .5;
    auto xVelocity = xInput * Constants::SwerveDrive::kMaxLinearSpeed * inputVelocityLimit;
    auto yVelocity = yInput * Constants::SwerveDrive::kMaxLinearSpeed * inputVelocityLimit;

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
                                    Edge.targetPose.Rotation().Radians());

    m_thetaErrorAbs = std::abs(
        (currentPose.Rotation() - Edge.targetPose.Rotation()).Radians().value());

    if (m_thetaErrorAbs < m_thetaController.GetPositionTolerance()) {
        thetaVelocity = 0.0;
    }
    frc::ChassisSpeeds robotSpeeds;
    int thingy;
    if(Edge.useXAxis && isAngleValid){
        robotSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        vx, yVelocity, units::radians_per_second_t{thetaVelocity},
        currentPose.Rotation());
        thingy = 1;
    }
    else if (isAngleValid){
        robotSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        xVelocity, vy, units::radians_per_second_t{thetaVelocity},
        currentPose.Rotation());
        thingy = 2;
    }else{
        robotSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        xVelocity, yVelocity, units::radians_per_second_t{thetaVelocity},
        currentPose.Rotation());
        thingy = 3;
    }

    Logger::Instance().Log("AlignToEdge/thingy", thingy);
    Logger::Instance().Log("AlignToEdge/angleDifference", angleDifference.value());
    Logger::Instance().Log("AlignToEdge/normalizedAngle", normalizedAngle.value());
    Logger::Instance().Log("AlignToEdge/normalizedDesiredAngle", normalizedDesiredAngle.value());
    Logger::Instance().Log("AlignToEdge/isAngleValid", isAngleValid);
    Logger::Instance().Log("AlignToEdge/poseOnField", poseOnField);
    Logger::Instance().Log("AlignToEdge/leftIntakeDistanceFromRedHub", leftIntakeDistanceFromRedHub);
    Logger::Instance().Log("AlignToEdge/isValidForBlueBump", isValidForBlueBump);
    Logger::Instance().Log("AlignToEdge/isValidForRedBump", isValidForRedBump);
    Logger::Instance().Log("AlignToEdge/minDistance", Edge.minDistance.value());
    Logger::Instance().Log("AlignToEdge/xMinDistance", Edge.xMinDistance.value());
    Logger::Instance().Log("AlignToEdge/yMinDistance", Edge.yMinDistance.value());
    Logger::Instance().Log("AlignToEdge/targetPose", Edge.targetPose);
    Logger::Instance().Log("AlignToEdge/useXAxis", Edge.useXAxis);
    Logger::Instance().Log("AlignToEdge/direction", direction);
    Logger::Instance().Log("AlignToEdge/xInput", xInput);
    Logger::Instance().Log("AlignToEdge/yInput", yInput);
    Logger::Instance().Log("AlignToEdge/angleToFaceWall", angleToFaceWall);
    Logger::Instance().Log("AlignToEdge/currentPose", currentPose);
    Logger::Instance().Log("AlignToEdge/targetPose",Edge.targetPose);
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
