// Team 5687 2026

#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include "subsystem/drive/DriveSubsystem.h"

class AutoAlignToPoseCommand
    : public frc2::CommandHelper<frc2::Command, AutoAlignToPoseCommand> {
public:
  AutoAlignToPoseCommand(DriveSubsystem *driveSubsystem,
                         std::function<frc::Pose2d()> targetPoseFunction,
                         double constraintFactor = 1.0);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  DriveSubsystem *m_driveSubsystem;
  std::function<frc::Pose2d()> m_targetPoseFunction;

  frc::ProfiledPIDController<units::meters> m_driveController;
  frc::ProfiledPIDController<units::radians> m_thetaController;

  double m_driveErrorAbs{0.0};
  double m_thetaErrorAbs{0.0};

  // Feedforward fades from 1 (at ffMaxRadius and beyond) to 0 (at ffMinRadius
  // and closer), preventing the ff term from fighting the PID near the goal.
  static constexpr units::meter_t kFfMinRadius = 0.0_m;
  static constexpr units::meter_t kFfMaxRadius = 0.1_m;
};
