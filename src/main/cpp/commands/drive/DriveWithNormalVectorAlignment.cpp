// Team 5687 2026

// DriveWithNormalVectorAlignment.cpp
#include "commands/drive/DriveWithNormalVectorAlignment.h"

#include <frc/MathUtil.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

#include "subsystem/drive/DriveSubsystem.h"

using namespace Constants::DriveWithNormalVectorAlignment;

DriveWithNormalVectorAlignment::DriveWithNormalVectorAlignment(
    DriveSubsystem* drive, std::function<frc::Pose2d()> finalPoseSupplier,
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
  // Enable continuous input for rotation controller (-180 to 180 degrees)
  m_rotationController.EnableContinuousInput(std::numbers::pi,
                                             std::numbers::pi);

  AddRequirements(drive);
}

void DriveWithNormalVectorAlignment::Initialize() {
  // Get current robot-relative speeds and convert to field-relative
  frc::ChassisSpeeds robotRelativeSpeeds = m_drive->GetChassisSpeeds();
  m_fieldRelativeVelocity = frc::ChassisSpeeds::FromRobotRelativeSpeeds(
      robotRelativeSpeeds, m_drive->GetPose().Rotation());

  // Reset PID controllers
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

  // Calculate alignment pose - offset opposite to the normal vector
  double offsetX =
      -m_normalVectorOffset.value() * m_finalPose->Rotation().Cos();
  double offsetY =
      -m_normalVectorOffset.value() * m_finalPose->Rotation().Sin();

  m_alignmentPose = frc::Pose2d{m_finalPose->X() + units::meter_t{offsetX},
                                m_finalPose->Y() + units::meter_t{offsetY},
                                m_finalPose->Rotation()};
}

frc::Pose2d DriveWithNormalVectorAlignment::GetCurrentTargetPose() {
  UpdateTargetPoses();

  if (!m_alignmentPose.has_value() || !m_finalPose.has_value()) {
    return m_drive->GetPose();  // Return current pose if targets not set
  }

  frc::Pose2d robotPose = m_drive->GetPose();

  // Vector from robot to alignment pose
  frc::Translation2d robotToAlignment{m_alignmentPose->X() - robotPose.X(),
                                      m_alignmentPose->Y() - robotPose.Y()};

  // Approach vector from alignment pose to final pose
  frc::Translation2d approachVector{m_finalPose->X() - m_alignmentPose->X(),
                                    m_finalPose->Y() - m_alignmentPose->Y()};

  units::meter_t approachMagnitude = approachVector.Norm();

  if (approachMagnitude.value() < 0.001) {
    return *m_finalPose;  // Already at final pose
  }

  frc::Translation2d approachDirection =
      approachVector / approachMagnitude.value();

  // Project robot position onto alignment line
  double robotProjection =
      robotToAlignment.X().value() * -approachDirection.X().value() +
      robotToAlignment.Y().value() * -approachDirection.Y().value();

  frc::Translation2d projectedPoint{
      robotPose.X() +
          units::meter_t{robotProjection * -approachDirection.X().value()},
      robotPose.Y() +
          units::meter_t{robotProjection * -approachDirection.Y().value()}};

  // Calculate perpendicular distance from alignment line
  units::meter_t perpendicularDistance =
      frc::Translation2d{projectedPoint.X() - m_alignmentPose->X(),
                         projectedPoint.Y() - m_alignmentPose->Y()}
          .Norm();

  // Calculate blend factor
  double blendFactor = 0.0;
  if (perpendicularDistance > 0_m) {
    blendFactor = frc::ApplyDeadband(
        1.0 - (perpendicularDistance - m_blendEndDistance).value() /
                  (m_blendStartDistance - m_blendEndDistance).value(),
        0.0);
    blendFactor = std::clamp(blendFactor, 0.0, 1.0);
  }

  // Interpolate between alignment pose and final pose
  units::meter_t targetX =
      m_alignmentPose->X() +
      units::meter_t{blendFactor * approachVector.X().value()};
  units::meter_t targetY =
      m_alignmentPose->Y() +
      units::meter_t{blendFactor * approachVector.Y().value()};

  return frc::Pose2d{targetX, targetY, m_finalPose->Rotation()};
}

void DriveWithNormalVectorAlignment::UpdatePIDGains() {
  m_xController.SetPID(kDriveKp, kDriveKi, kDriveKd);
  m_yController.SetPID(kDriveKp, kDriveKi, kDriveKd);
  m_rotationController.SetPID(kRotKp, kRotKi, kRotKd);
}

void DriveWithNormalVectorAlignment::UpdatePIDTolerances() {
  m_xController.SetTolerance(m_positionTolerance.value(),
                             m_velocityTolerance.value());
  m_yController.SetTolerance(m_positionTolerance.value(),
                             m_velocityTolerance.value());
  m_rotationController.SetTolerance(units::degree_t{2.0}.value(),
                                    units::degrees_per_second_t{2.0}.value());
}

void DriveWithNormalVectorAlignment::Execute() {
  frc::Pose2d targetPose = GetCurrentTargetPose();
  frc::Pose2d currentPose = m_drive->GetPose();

  // Calculate error vector
  frc::Translation2d errorVec{targetPose.X() - currentPose.X(),
                              targetPose.Y() - currentPose.Y()};

  // Current velocity vector
  frc::Translation2d currentVel{
      units::meter_t{m_fieldRelativeVelocity.vx.value()},
      units::meter_t{m_fieldRelativeVelocity.vy.value()}};

  // Check if moving away from target
  double velocityDot = errorVec.X().value() * currentVel.X().value() +
                       errorVec.Y().value() * currentVel.Y().value();
  bool movingAway = velocityDot < 0;

  // Calculate desired velocities from PID
  double vxDesired =
      m_xController.Calculate(currentPose.X().value(), targetPose.X().value());
  double vyDesired =
      m_yController.Calculate(currentPose.Y().value(), targetPose.Y().value());

  // Apply counteraction if moving away
  if (movingAway) {
    frc::Translation2d counteract = currentVel * -m_counteractGain;
    vxDesired += counteract.X().value();
    vyDesired += counteract.Y().value();
  }

  frc::Translation2d desiredVel{units::meter_t{vxDesired},
                                units::meter_t{vyDesired}};

  units::meter_t currentDistance = errorVec.Norm();

  // Limit speed based on distance (velocity profile)
  double maxAllowedSpeed =
      std::sqrt(2.0 * m_maxAcceleration.value() * currentDistance.value());
  maxAllowedSpeed = std::min(maxAllowedSpeed, m_maxVelocity.value());

  if (desiredVel.Norm().value() > maxAllowedSpeed) {
    desiredVel = desiredVel * (maxAllowedSpeed / desiredVel.Norm().value());
    vxDesired = desiredVel.X().value();
    vyDesired = desiredVel.Y().value();
  }

  // Maintain minimum velocity when far from target
  if (currentDistance > m_positionTolerance * 2.0) {
    double currentSpeed = desiredVel.Norm().value();
    double minSpeed = m_minOutput * m_maxVelocity.value();

    if (currentSpeed < minSpeed) {
      desiredVel = errorVec * (minSpeed / currentDistance.value());
      vxDesired = desiredVel.X().value();
      vyDesired = desiredVel.Y().value();
    }
  }

  // Calculate desired rotation
  double omegaDesired =
      m_rotationController.Calculate(currentPose.Rotation().Radians().value(),
                                     targetPose.Rotation().Radians().value());

  // Smooth rotation
  double omega = std::lerp(m_fieldRelativeVelocity.omega.value(), omegaDesired,
                           m_smoothingFactor);

  // Apply velocity smoothing with acceleration limiting
  constexpr units::second_t dt = 20_ms;  // Assuming 50Hz update rate

  frc::Translation2d deltaV{
      units::meter_t{(vxDesired - m_fieldRelativeVelocity.vx.value()) *
                     m_smoothingFactor},
      units::meter_t{(vyDesired - m_fieldRelativeVelocity.vy.value()) *
                     m_smoothingFactor}};

  // Limit acceleration
  double effectiveMaxAccel =
      movingAway ? m_maxAcceleration.value() * m_aggressiveAccelMultiplier
                 : m_maxAcceleration.value();

  double maxDv = effectiveMaxAccel * dt.value();
  if (deltaV.Norm().value() > maxDv) {
    deltaV = deltaV * (maxDv / deltaV.Norm().value());
  }

  // Update field-relative velocity
  m_fieldRelativeVelocity = frc::ChassisSpeeds{
      units::meters_per_second_t{m_fieldRelativeVelocity.vx.value() +
                                 deltaV.X().value()},
      units::meters_per_second_t{m_fieldRelativeVelocity.vy.value() +
                                 deltaV.Y().value()},
      units::radians_per_second_t{omega}};

  // Convert to robot-relative and send to drive
  m_drive->DriveFieldRelative(m_fieldRelativeVelocity);
}

void DriveWithNormalVectorAlignment::End(bool interrupted) {
  m_drive->Drive(frc::ChassisSpeeds{});
}

bool DriveWithNormalVectorAlignment::IsFinished() {
  if (!m_finalPose.has_value()) {
    return false;
  }

  // Check if at setpoint
  bool atFinalPose = m_xController.AtSetpoint() && m_yController.AtSetpoint() &&
                     m_rotationController.AtSetpoint() &&
                     std::abs(m_fieldRelativeVelocity.vx.value()) <
                         m_velocityTolerance.value() &&
                     std::abs(m_fieldRelativeVelocity.vy.value()) <
                         m_velocityTolerance.value() &&
                     std::abs(m_fieldRelativeVelocity.omega.value()) <
                         units::degrees_per_second_t{2.0}.value();

  // Check distance to final pose
  frc::Pose2d robotPose = m_drive->GetPose();
  units::meter_t distanceToFinalPose =
      frc::Translation2d{robotPose.X() - m_finalPose->X(),
                         robotPose.Y() - m_finalPose->Y()}
          .Norm();

  return distanceToFinalPose < m_positionTolerance * 3.0 && atFinalPose;
}
