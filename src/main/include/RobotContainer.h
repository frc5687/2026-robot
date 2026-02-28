// Team 5687 2026

// RobotContainer.h
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandPS5Controller.h>
#include <units/angle.h>

#include "subsystem/drive/DriveSubsystem.h"
#include "subsystem/feeder/FeederSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/hood/HoodSubsystem.h"
#include "subsystem/intake/IntakeSystem.h"
#include "subsystem/intake/bottomroller/IntakeBottomRollerSubsystem.h"
#include "subsystem/intake/deployer/IntakeDeployerSubsystem.h"
#include "subsystem/intake/toproller/IntakeTopRollerSubsystem.h"
#include "subsystem/kicker/KickerSubsystem.h"
#include "subsystem/shooter/ShooterSystem.h"
#include "subsystem/vision/VisionSubsystem.h"
#include "utils/TunableDouble.h"
#include "viz/RobotViz.h"

class RobotContainer {
public:
  RobotContainer();
  frc2::CommandPtr GetAutonomousCommand();
  void Periodic();

private:
  void ConfigureBindings();
  void ConfigureAutoCommands();

  DriveSubsystem m_drive;
  FlywheelSubsystem m_flywheel;
  HoodSubsystem m_hood;
  FeederSubsystem m_feeder;
  KickerSubsystem m_kicker;
  IntakeDeployerSubsystem m_intakeDeployer;
  IntakeTopRollerSubsystem m_intakeTopRoller;
  IntakeBottomRollerSubsystem m_intakeBottomRoller;
  VisionSubsystem m_vision;
  IntakeSystem m_intake;
  ShooterSystem m_shooter;
  RobotViz m_robotViz{};
  frc2::CommandPS5Controller m_driver{0};
};
