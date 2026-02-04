// RobotContainer.h
#pragma once

#include <frc2/command/button/CommandPS5Controller.h>
#include <frc2/command/CommandPtr.h>
#include <units/angle.h>

#include <array>
#include <memory>

#include "subsystem/drive/DriveSubsystem.h"
#include "subsystem/intake/IntakeRoller.h"
#include "subsystem/intake/CTREIntakeRollerIO.h"
#include "subsystem/intake/IntakeSubsystem.h"
#include "subsystem/intake/linearintake/LinearIntake.h"
#include "subsystem/vision/VisionSubsystem.h"

#include "subsystem/intake/IntakeRoller.h"
#include "subsystem/intake/CTREIntakeRollerIO.h"
#include "subsystem/intake/linearintake/LinearIntake.h"
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();
  
  std::unique_ptr<DriveSubsystem> CreateDrive();
  // std::unique_ptr<VisionSubsystem> CreateVision();

  std::unique_ptr<VisionSubsystem> CreateVision();


  std::unique_ptr<DriveSubsystem> m_drive;

  std::unique_ptr<LinearIntake> CreateLinearIntake();
  std::unique_ptr<LinearIntake> m_linearIntake;
  
  std::unique_ptr<IntakeRoller> CreateIntakeRoller();
  std::unique_ptr<IntakeRoller> m_intakeRoller;
  
  std::unique_ptr<IntakeSubsystem> CreateIntakeSubsystem();
  std::unique_ptr<IntakeSubsystem>m_intakeSubsystem;

  std::unique_ptr<VisionSubsystem> m_vision;

  // std::unique_ptr<VisionSubsystem> m_vision;
  frc2::CommandPS5Controller m_driver{0};


};