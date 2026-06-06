// Team 5687 2026

#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/voltage.h>

#include <functional>

#include "subsystem/drive/DriveSubsystem.h"
#include "subsystem/drive/SwerveDriveConstants.h"
#include "subsystem/feeder/FeederSubsystem.h"
#include "subsystem/floor/FloorSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/hood/HoodSubsystem.h"
#include "subsystem/intake/bottomroller/IntakeBottomRollerSubsystem.h"
#include "subsystem/intake/deployer/IntakeDeployerSubsystem.h"
#include "subsystem/intake/toproller/IntakeTopRollerSubsystem.h"
#include "subsystem/shooter/ShotCalculator.h"
#include "utils/TunableDouble.h"

class ShootCommand : public frc2::CommandHelper<frc2::Command, ShootCommand> {
public:
  ShootCommand(DriveSubsystem *drive, FlywheelSubsystem *flywheel,
               HoodSubsystem *hood, IntakeTopRollerSubsystem *topRoller,
               IntakeBottomRollerSubsystem *bottomRoller,
               FeederSubsystem *feeder, FloorSubsystem *floor,
               IntakeDeployerSubsystem *deployer,
               std::function<double()> throttle,
               std::function<double()> strafe);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  // Shoot sequence phases.
  enum class Phase {
    kIdle,         // wait for solution
    kPreClear,     // clear feeder path
    kExtendHold,   // initial feed window
    kQuickRetract, // short retract pulse
    kReExtend,     // extend again
    kSlowRetract,  // final retract
  };

  void ResetSequenceState();
  void UpdateFlywheelAndHood(const ShotSolution &solution, units::second_t now);
  void UpdateDrive(const ShotSolution &solution);
  void UpdateSequence(units::second_t now, bool solutionReady);
  void EnterPhase(Phase phase, units::second_t now);
  double BurstOffset(units::second_t now) const;
  bool FeedersShouldRun(units::second_t now) const;
  void RunFeeders();

  DriveSubsystem *m_drive;
  FlywheelSubsystem *m_flywheel;
  HoodSubsystem *m_hood;
  IntakeTopRollerSubsystem *m_topRoller;
  IntakeBottomRollerSubsystem *m_bottomRoller;
  FeederSubsystem *m_feeder;
  FloorSubsystem *m_floor;
  IntakeDeployerSubsystem *m_deployer;

  std::function<double()> m_throttle;
  std::function<double()> m_strafe;

  ShotCalculator m_shotCalculator;

  frc::PIDController m_headingController{
      Constants::SwerveDrive::Shooting::kAimkP, 0.0,
      Constants::SwerveDrive::Shooting::kAimkD};

  static constexpr units::volt_t kFloorVoltage = 12_V;
  static constexpr units::volt_t kFeederVoltage = 12_V;
  static constexpr units::volt_t kTopVoltage = 6_V;
  static constexpr units::volt_t kBottomVoltage = 10_V;
  static constexpr units::volt_t kBackoffFloorVoltage = -2.0_V;
  static constexpr units::volt_t kPreclearFlywheelReverseVoltage = -1.5_V;

  static constexpr units::second_t kInitialExtendDuration = 1.5_s;
  static constexpr units::second_t kPulseRetractDuration = 0.2_s;
  static constexpr units::second_t kFinalRetractDuration = 0.6_s;

  // Burst timing and offset.
  static constexpr bool kEnableBurst = true;
  static constexpr units::second_t kFeederStartDelay = 0.2_s;
  static constexpr units::second_t kBurstRampDuration = 0.25_s;
  static constexpr double kBurstRpmOffset = 100.0;
  static_assert(kFeederStartDelay + kBurstRampDuration < kInitialExtendDuration,
                "Burst profile must fit within the initial extend hold");

  static constexpr double kDeadband = 0.1;

  Phase m_phase{Phase::kIdle};
  units::second_t m_phaseStartTime{0_s};
  bool m_hasFedFuel{false};
  TunableDouble rmpBumpTune{"rpmBump", 0.0};
};
