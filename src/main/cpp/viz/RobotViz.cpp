// Team 5687 2026

#include "viz/RobotViz.h"

#include <frc/geometry/Rotation3d.h>

#include "Constants.h"
#include "frc/Timer.h"

frc::Pose3d RobotViz::CalculateTurretPoseLeft(const frc::Pose2d& drivePose,
                                          const TurretState& turretState) {
  frc::Pose3d drive3d{drivePose};

  auto turretMountPose =
      drive3d.TransformBy(Constants::Geometry::kRobotToTurretLeft);

  frc::Transform3d turretRotation{{0_m, 0_m, 0_m},
                                  {0_rad, 0_rad, turretState.angle}};
  return turretMountPose.TransformBy(turretRotation);
}

frc::Pose3d RobotViz::CalculateTurretPoseRight(const frc::Pose2d& drivePose,
                                          const TurretState& turretState) {
  frc::Pose3d drive3d{drivePose};

  auto turretMountPose =
      drive3d.TransformBy(Constants::Geometry::kRobotToTurretRight);

  frc::Transform3d turretRotation{{0_m, 0_m, 0_m},
                                  {0_rad, 0_rad, turretState.angle}};
  return turretMountPose.TransformBy(turretRotation);
}

void RobotViz::Update() {
  units::second_t timestamp = frc::Timer::GetFPGATimestamp();
  auto driveState = m_robotState.GetDriveState(timestamp);
  auto turretState = m_robotState.GetTurretState(timestamp);
  auto turretPoseLeft = CalculateTurretPoseLeft(driveState.pose, turretState);
  auto turretPoseRight = CalculateTurretPoseRight(driveState.pose, turretState);
  const std::array<frc::Pose3d, 2> components{
    turretPoseLeft,
    turretPoseRight
  };
  Log("DrivePose", driveState.pose);
  Log("DrivePose3d", frc::Pose3d{driveState.pose});
  Log("Components", components);
  Log("Turret", turretPoseLeft);
  // Log("TurretPose", turretPose);
}

void RobotViz::FutureViz(units::second_t futureDt) {
  units::second_t timestamp = frc::Timer::GetFPGATimestamp() + futureDt;
  auto driveState = m_robotState.GetDriveState(timestamp);
  auto turretState = m_robotState.GetTurretState(timestamp);
  auto turretPoseLeft = CalculateTurretPoseLeft(driveState.pose, turretState);
  auto turretPoseRight = CalculateTurretPoseRight(driveState.pose, turretState);
  const std::array<frc::Pose3d, 2> components{
    turretPoseLeft,
    turretPoseRight,
  };
  Log("Future/DrivePose", driveState.pose);
  Log("Future/Components", components);
}
