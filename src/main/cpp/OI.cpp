// Team 5687 2026

#include "OI.h"

#include <frc2/command/Commands.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include "commands/drive/DriveMaintainingHeadingCommand.h"
#include "subsystem/hood/HoodSubsystem.h"
#include "subsystem/shooter/ShooterSystem.h"
#include "units/angle.h"

OI::OI(DriveSubsystem& drive, FlywheelSubsystem& flywheel,
       IndexerSubsystem& indexer, IntakeSubsystem& intake,
       ShooterSystem& shooter, HoodSubsystem& hood):
         m_microseconds("microseconds", "microseconds", 1500)
 {
  ConfigureDriverBindings(drive, flywheel, indexer, intake, shooter, hood);
}

void OI::ConfigureDriverBindings(DriveSubsystem& drive,
                                  FlywheelSubsystem& flywheel,
                                  IndexerSubsystem& indexer,
                                  IntakeSubsystem& intake,
                                  ShooterSystem& shooter,
                                  HoodSubsystem& hood) {
  using frc2::cmd::Run;
  using frc2::cmd::RunEnd;

  drive.SetDefaultCommand(DriveMaintainingHeadingCommand(
      &drive,
      [this] { return -m_driver.GetLeftY(); },
      [this] { return -m_driver.GetLeftX(); },
      [this] { return -m_driver.GetRightX(); },
      /*slewLimiter=*/true));

  m_driver.L2().WhileTrue(
      RunEnd(
          [&intake] { intake.SetVoltage(10_V); },
          [&intake] { intake.SetVoltage(0_V); },
          {&intake}));

  m_driver.Circle().WhileTrue(
      Run([&indexer] { indexer.SetVoltage(10_V); }, {&indexer}));

  m_driver.Square().WhileTrue(
      Run([&flywheel] { flywheel.SetRPM(2000_rpm, 2000_rpm); }, {&flywheel}));

  m_driver.Triangle().WhileTrue(
      Run([&shooter] { shooter.SetState(ShooterState::TRACKING); }));

        m_driver.R2().WhileTrue(
      Run([&hood, this] { hood.SetHoodPosition(units::turn_t{m_microseconds.Get()}, units::turn_t{0}); }, {&hood}));
}