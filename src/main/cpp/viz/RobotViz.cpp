// Team 5687 2026

#include "viz/RobotViz.h"

#include <frc/geometry/Rotation3d.h>

#include "frc/Timer.h"

void RobotViz::Update() {
  units::second_t timestamp = frc::Timer::GetFPGATimestamp();
  auto driveState = m_robotState.GetDriveState(timestamp);

  Log("DrivePose", driveState.estimatedPose);
}
