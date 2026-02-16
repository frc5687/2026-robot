// Team 5687 2026

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandPS5Controller.h>

#include <optional>

#include "RobotContainer.h"

/**
 * @brief Main robot class implementing swerve drive robot functionality.
 *
 * Manages robot lifecycle, subsystem initialization, and command scheduling.
 * Configures appropriate hardware IO implementations based on simulation vs
 * real robot.
 */
class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

 private:
  std::optional<frc2::CommandPtr> m_autonomousCommand;

  RobotContainer m_container;
};

#ifndef RUNNING_FRC_TESTS
int main();
#endif
