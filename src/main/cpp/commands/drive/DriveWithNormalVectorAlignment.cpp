// DriveWithNormalVectorAlignment.cpp
#include "commands/drive/DriveWithNormalVectorAlignment.h"

#include <frc/MathUtil.h>

#include <cmath>
#include <numbers>

#include "subsystem/drive/DriveSubsystem.h"

using namespace Constants::DriveWithNormalVectorAlignment;

DriveWithNormalVectorAlignment::DriveWithNormalVectorAlignment(
    DriveSubsystem* drive,
    std::function<frc::Pose2d()> finalPoseSupplier,
    bool isAlgae)
    : m_drive(drive),
      m_finalPoseSupplier(std::move(finalPoseSupplier)),
      m_xController(kDriveKp, kDriveKi, kDriveKd),
      m_yController(kDriveKp, kDriveKi, kDriveKd),
      m_rotationController(kRotKp, kRotKi, kRotKd),
      m_normalVectorOffset(kNormalVectorOffset),
      m_blendStartDistance(isAlgae ? kBlendStartAlgae : kBlendStart),
      m_blendEndDistance(kBlendEnd),
      m_maxVelocity(kMaxVelocity),
      m_maxAcceleration(kMaxAcceleration),
      m_positionTolerance(kPositionTolerance),
      m_velocityTolerance(kVelocityTolerance),
      m_minOutput(kMinOutput),
      m_counteractGain(kCounteractGain),
      m_aggressiveAccelMultiplier(kAggressiveAccelMultiplier),
      m_smoothingFactor(kSmoothingFactor),
      m_isAlgae(isAlgae) {
  
  m_rotationController.EnableContinuousInput(
     std::numbers::pi, 
      std::numbers::pi);

  AddRequirements(drive);
}

void DriveWithNormalVectorAlignment::Initialize() {
  frc::ChassisSpeeds robotRelativeSpeeds = m_drive->GetChassisSpeeds();
  m_fieldRelativeVelocity = frc::ChassisSpeeds::FromRobotRelativeSpeeds(
      robotRelativeSpeeds, m_drive->GetPose().Rotation());

  m_xController.Reset();
  m_yController.Reset();
  m_rotationController.Reset();

  UpdatePIDTolerances();
  UpdateTargetPoses();
}

void DriveWithNormalVectorAlignment::UpdateTargetPoses() {
  m_finalPose = m_finalPoseSupplier();

  if (!m_finalPose.has_value()) {
    return;
  }

  double offsetX = -m_normalVectorOffset.value() * 
                   m_finalPose->Rotation().Cos();
  double offsetY = -m_normalVectorOffset.value() * 
                   m_finalPose->Rotation().Sin();

  m_alignmentPose = frc::Pose2d{
      m_finalPose->X() + units::meter_t{offsetX},
      m_finalPose->Y() + units::meter_t{offsetY},
      m_finalPose->Rotation()};
}

frc::Pose2d DriveWithNormalVectorAlignment::GetCurrentTargetPose() {
  UpdateTargetPoses();

  if (!m_alignmentPose.has_value() || !m_finalPose.has_value()) {
    return m_drive->GetPose(); 
  }

  frc::Pose2d robotPose = m_drive->GetPose();

  frc::Translation2d robotToAlignment{
      m_alignmentPose->X() - robotPose.X(),
      m_alignmentPose->Y() - robotPose.Y()};

  frc::Translation2d approachVector{
      m_finalPose->X() - m_alignmentPose->X(),
      m_finalPose->Y() - m_alignmentPose->Y()};

  units::meter_t approachMagnitude = approachVector.Norm();
  
  if (approachMagnitude.value() < 0.001) {
    return *m_finalPose;  // Already at final pose
  }

  frc::Translation2d approachDirection = approachVector / approachMagnitude.value();

  double robotProjection = 
      robotToAlignment.X().value() * -approachDirection.X().value() +
      robotToAlignment.Y().value() * -approachDirection.Y().value();

  frc::Translation2d projectedPoint{
      robotPose.X() + units::meter_t{robotProjection * -approachDirection.X().value()},
      robotPose.Y() + units::meter_t{robotProjection * -approachDirection.Y().value()}};

  units::meter_t perpendicularDistance = 
      frc::Translation2d{
          projectedPoint.X() - m_alignmentPose->X(),
          projectedPoint.Y() - m_alignmentPose->Y()}.Norm();

  double blendFactor = 0.0;
  if (perpendicularDistance > 0_m) {
    blendFactor = frc::ApplyDeadband(
        1.0 - (perpendicularDistance - m_blendEndDistance).value() / 
              (m_blendStartDistance - m_blendEndDistance).value(),
        0.0);
    blendFactor = std::clamp(blendFactor, 0.0, 1.0);
  }

  units::meter_t targetX = m_alignmentPose->X() + 
                           units::meter_t{blendFactor * approachVector.X().value()};
  units::meter_t targetY = m_alignmentPose->Y() + 
                           units::meter_t{blendFactor * approachVector.Y().value()};

  return frc::Pose2d{targetX, targetY, m_finalPose->Rotation()};
}

void DriveWithNormalVectorAlignment::UpdatePIDGains() {
  m_xController.SetPID(kDriveKp, kDriveKi, kDriveKd);
  m_yController.SetPID(kDriveKp, kDriveKi, kDriveKd);
  m_rotationController.SetPID(kRotKp, kRotKi, kRotKd);
}

void DriveWithNormalVectorAlignment::UpdatePIDTolerances() {
  m_xController.SetTolerance(
      m_positionTolerance.value(), 
      m_velocityTolerance.value());
  m_yController.SetTolerance(
      m_positionTolerance.value(), 
      m_velocityTolerance.value());
  m_rotationController.SetTolerance(
      units::degree_t{2.0}.value(), 
      units::degrees_per_second_t{2.0}.value());
}

void DriveWithNormalVectorAlignment::Execute() {
  frc::Pose2d targetPose = GetCurrentTargetPose();
  frc::Pose2d currentPose = m_drive->GetPose();

  frc::Translation2d errorVec{
      targetPose.X() - currentPose.X(),
      targetPose.Y() - currentPose.Y()};

  frc::Translation2d currentVel{
      units::meter_t{m_fieldRelativeVelocity.vx.value()},
      units::meter_t{m_fieldRelativeVelocity.vy.value()}};

  double velocityDot = 
      errorVec.X().value() * currentVel.X().value() +
      errorVec.Y().value() * currentVel.Y().value();
  bool movingAway = velocityDot < 0;

  double vxDesired = m_xController.Calculate(
      currentPose.X().value(), targetPose.X().value());
  double vyDesired = m_yController.Calculate(
      currentPose.Y().value(), targetPose.Y().value());

  if (movingAway) {
    frc::Translation2d counteract = currentVel * -m_counteractGain;
    vxDesired += counteract.X().value();
    vyDesired += counteract.Y().value();
  }

  frc::Translation2d desiredVel{
      units::meter_t{vxDesired}, 
      units::meter_t{vyDesired}};

  units::meter_t currentDistance = errorVec.Norm();

  double maxAllowedSpeed = std::sqrt(
      2.0 * m_maxAcceleration.value() * currentDistance.value());
  maxAllowedSpeed = std::min(maxAllowedSpeed, m_maxVelocity.value());

  if (desiredVel.Norm().value() > maxAllowedSpeed) {
    desiredVel = desiredVel * (maxAllowedSpeed / desiredVel.Norm().value());
    vxDesired = desiredVel.X().value();
    vyDesired = desiredVel.Y().value();
  }

  if (currentDistance > m_positionTolerance * 2.0) {
    double currentSpeed = desiredVel.Norm().value();
    double minSpeed = m_minOutput * m_maxVelocity.value();
    
    if (currentSpeed < minSpeed) {
      desiredVel = errorVec * (minSpeed / currentDistance.value());
      vxDesired = desiredVel.X().value();
      vyDesired = desiredVel.Y().value();
    }
  }

  double omegaDesired = m_rotationController.Calculate(
      currentPose.Rotation().Radians().value(),
      targetPose.Rotation().Radians().value());

  double omega = std::lerp(
      m_fieldRelativeVelocity.omega.value(),
      omegaDesired,
      m_smoothingFactor);

  constexpr units::second_t dt = 20_ms; 

  frc::Translation2d deltaV{
      units::meter_t{(vxDesired - m_fieldRelativeVelocity.vx.value()) * m_smoothingFactor},
      units::meter_t{(vyDesired - m_fieldRelativeVelocity.vy.value()) * m_smoothingFactor}};

  double effectiveMaxAccel = movingAway
      ? m_maxAcceleration.value() * m_aggressiveAccelMultiplier
      : m_maxAcceleration.value();

  double maxDv = effectiveMaxAccel * dt.value();
  if (deltaV.Norm().value() > maxDv) {
    deltaV = deltaV * (maxDv / deltaV.Norm().value());
  }

  m_fieldRelativeVelocity = frc::ChassisSpeeds{
      units::meters_per_second_t{m_fieldRelativeVelocity.vx.value() + deltaV.X().value()},
      units::meters_per_second_t{m_fieldRelativeVelocity.vy.value() + deltaV.Y().value()},
      units::radians_per_second_t{omega}};

  m_drive->DriveFieldRelative(m_fieldRelativeVelocity);
}

void DriveWithNormalVectorAlignment::End(bool interrupted) {
  m_drive->Drive(frc::ChassisSpeeds{});
}

bool DriveWithNormalVectorAlignment::IsFinished() {
  if (!m_finalPose.has_value()) {
    return false;
  }

  bool atFinalPose = 
      m_xController.AtSetpoint() &&
      m_yController.AtSetpoint() &&
      m_rotationController.AtSetpoint() &&
      std::abs(m_fieldRelativeVelocity.vx.value()) < m_velocityTolerance.value() &&
      std::abs(m_fieldRelativeVelocity.vy.value()) < m_velocityTolerance.value() &&
      std::abs(m_fieldRelativeVelocity.omega.value()) < 
          units::degrees_per_second_t{2.0}.value();

  frc::Pose2d robotPose = m_drive->GetPose();
  units::meter_t distanceToFinalPose = frc::Translation2d{
      robotPose.X() - m_finalPose->X(),
      robotPose.Y() - m_finalPose->Y()}.Norm();

  return distanceToFinalPose < m_positionTolerance * 3.0 && atFinalPose;
}