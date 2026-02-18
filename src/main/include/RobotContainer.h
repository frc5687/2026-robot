// Team 5687 2026

// RobotContainer.h
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandPS5Controller.h>
#include <units/angle.h>

#include <memory>

#include "subsystem/drive/DriveSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/turret/TurretSubsystem.h"
#include "subsystem/vision/VisionSubsystem.h"
#include "subsystem/hood/HoodSubsystem.h"
#include "subsystem/shooter/ShooterSystem.h"

#include "viz/RobotViz.h"

class RobotContainer {
 public:
  RobotContainer();
  frc2::CommandPtr GetAutonomousCommand();
  void Periodic();

 private:
  void ConfigureBindings();

  std::unique_ptr<DriveSubsystem> CreateDrive();
  std::unique_ptr<TurretSubsystem> CreateTurret();
  std::unique_ptr<FlywheelSubsystem> CreateFlywheel();
  std::unique_ptr<HoodSubsystem> CreateHood();
  std::unique_ptr<VisionSubsystem> CreateVision();


  std::unique_ptr<DriveSubsystem> m_drive;
  std::unique_ptr<TurretSubsystem> m_turret;
  std::unique_ptr<FlywheelSubsystem> m_flywheel;
  std::unique_ptr<HoodSubsystem> m_hood;
  //std::unique_ptr<VisionSubsystem> m_vision;

  std::unique_ptr<ShooterSystem> m_shooter;

  RobotViz m_robotViz{};
  frc2::CommandPS5Controller m_driver{0};
};
