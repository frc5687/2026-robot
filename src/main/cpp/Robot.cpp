// Team 5687 2026

#include "Robot.h"

#include <frc/DriverStation.h>
#include <frc/Notifier.h>
#include <frc/RobotBase.h>
#include <frc/Threads.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <wpi/raw_ostream.h>

#include <ctre/phoenix6/SignalLogger.hpp>

#include "subsystem/CoordinatedSystemManager.h"
#include "utils/MatchTracker.h"

Robot::Robot() {
  frc::LiveWindow::DisableAllTelemetry();
  ctre::phoenix6::SignalLogger::EnableAutoLogging(false);
  frc::DriverStation::SilenceJoystickConnectionWarning(true);

  if (!frc::Notifier::SetHALThreadPriority(true, 40)) {
    wpi::errs() << "Failed to set HAL notifier thread priority\n";
  }

  // 2) Main robot loop thread
  if (!frc::SetCurrentThreadPriority(true, 15)) {
    wpi::errs() << "Failed to set main robot thread priority\n";
  }
}

void Robot::RobotPeriodic() {
  auto startTime = frc::Timer::GetFPGATimestamp();
  MatchTracker::Instance().UpdateAllianceState();

  frc2::CommandScheduler::GetInstance().Run();
  m_container.Periodic();
  CoordinatedSystemManager::Instance().UpdateAll();

  auto endTime = frc::Timer::GetFPGATimestamp();
  MatchTracker::Instance().LogState(endTime);
  auto updateTime = (endTime - startTime);
  Logger::Instance().Log("Performance/UpdateTime", updateTime.value() * 1000);
}

void Robot::DisabledInit() { MatchTracker::Instance().OnDisableInit(); }
void Robot::DisabledPeriodic() {}
void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  MatchTracker::Instance().OnAutonomousInit();
  m_autonomousCommand = m_container.GetAutonomousCommand();
  if (m_autonomousCommand != nullptr) {
    frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand);
  }
}

void Robot::AutonomousPeriodic() {}
void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  MatchTracker::Instance().OnTeleopInit();
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  MatchTracker::Instance().OnTestInit();
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}
void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
