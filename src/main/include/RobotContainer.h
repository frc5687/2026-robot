// Team 5687 2026

// RobotContainer.h
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandPS5Controller.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <units/angle.h>

#include "frc/smartdashboard/SendableChooser.h"
#include "subsystem/drive/DriveSubsystem.h"
#include "subsystem/feeder/FeederSubsystem.h"
#include "subsystem/floor/FloorSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/hood/HoodSubsystem.h"
#include "subsystem/intake/IntakeSystem.h"
#include "subsystem/intake/bottomroller/IntakeBottomRollerSubsystem.h"
#include "subsystem/intake/deployer/IntakeDeployerSubsystem.h"
#include "subsystem/intake/toproller/IntakeTopRollerSubsystem.h"
#include "subsystem/lights/LightsSubsystem.h"
#include "subsystem/shooter/ShooterSystem.h"
#include "subsystem/vision/VisionSubsystem.h"
#include "utils/SubsystemBatteryLogger.h"
#include "viz/RobotViz.h"

class RobotContainer {
public:
  RobotContainer();
  frc2::Command *GetAutonomousCommand();
  void SetAutoDriveCurrentLimits();
  void SetTeleopDriveCurrentLimits();
  void Periodic();

private:
  void ConfigureBindings();
  void ConfigureAutoCommands();

  DriveSubsystem m_drive;
  FlywheelSubsystem m_flywheel;
  HoodSubsystem m_hood;
  FeederSubsystem m_feeder;
  FloorSubsystem m_floor;
  IntakeDeployerSubsystem m_intakeDeployer;
  IntakeTopRollerSubsystem m_intakeTopRoller;
  IntakeBottomRollerSubsystem m_intakeBottomRoller;
  VisionSubsystem m_vision;
  IntakeSystem m_intake;
  ShooterSystem m_shooter;
  RobotViz m_robotViz{};
  frc2::CommandPS5Controller m_driver{0};
  frc2::CommandPS5Controller m_operator{1};
  // frc2::CommandPS5Controller m_debugger{3};
  frc::SendableChooser<frc2::Command *> m_autoChooser;
  SubsystemBatteryLogger m_batteryLogger;
};
