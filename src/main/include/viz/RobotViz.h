// Team 5687 2026

#pragma once

#include <frc/geometry/Pose3d.h>

#include <string>

#include "RobotState.h"
#include "utils/Logger.h"

// This will handle visualize the robot and other parameters like predicted
// shots.
class RobotViz {
 public:
  RobotViz() : m_robotState(RobotState::Instance()) {}
  void Update();
  frc::Pose3d CalculateTurretPoseLeft(const frc::Pose2d& drivePose,
                                  const TurretState& turretState);
  frc::Pose3d CalculateTurretPoseRight(const frc::Pose2d& drivePose,
                                  const TurretState& turretState);

  void FutureViz(units::second_t futureDt);

 private:
  RobotState& m_robotState;

  template <typename T>
  void Log(const std::string& key, const T& value) {
    Logger::Instance().Log("Vizualization/" + key, value);
  }
};
