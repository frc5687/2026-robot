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
         m_microseconds("rpm", "rpm", 1500),
         m_hoodangle("angle, angle", 0.0)
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

  m_driver.R2().WhileTrue(
      RunEnd([&indexer] { indexer.SetVoltage(10_V); }, [&indexer] {indexer.SetVoltage(0_V);},{&indexer}).AlongWith(RunEnd(
          [&intake] { intake.SetVoltage(10_V); },
          [&intake] { intake.SetVoltage(0_V); },
          {&intake})));

  m_driver.R2().WhileTrue(
    DriveMaintainingHeadingCommand(
      &drive,
      [this] { return -m_driver.GetLeftY() * 0.4; },
      [this] { return -m_driver.GetLeftX() * 0.4; },
      [this] { return -m_driver.GetRightX(); },
      /*slewLimiter=*/true).ToPtr()
  );

  m_driver.Options().OnTrue(Run([&drive] {drive.ResetHeading(0_deg);}));
  m_driver.L1().WhileTrue(
      Run([&shooter] { shooter.SetState(ShooterState::TRENCH); }));

    m_driver.R1().WhileTrue(
      Run([&shooter] { shooter.SetState(ShooterState::TRACKING); }));
 
    m_driver.Triangle().WhileTrue(
      Run([&shooter] { shooter.SetState(ShooterState::PASSING); }));

    m_driver.POVUp().WhileTrue(
      Run([&hood] { hood.SetMicroseconds(700); }));
     m_driver.POVDown().WhileTrue(
      Run([&hood] { hood.SetMicroseconds(2100); }));
    
    //     m_driver.Cross().OnTrue(
    //   Run([&hood, this] { hood.SetHoodPosition(units::turn_t{m_hoodangle.Get()}, units::turn_t{m_hoodangle.Get()}); }, {&hood}));
}