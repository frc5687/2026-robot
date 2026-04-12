// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/voltage.h>

#include "subsystem/feeder/FeederSubsystem.h"
#include "subsystem/floor/FloorSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/intake/bottomroller/IntakeBottomRollerSubsystem.h"
#include "subsystem/intake/deployer/IntakeDeployerSubsystem.h"
#include "subsystem/intake/toproller/IntakeTopRollerSubsystem.h"

class IntakeWithIndexCommand
    : public frc2::CommandHelper<frc2::Command, IntakeWithIndexCommand> {
public:
  IntakeWithIndexCommand(IntakeDeployerSubsystem *deployer,
                         IntakeTopRollerSubsystem *topRoller,
                         IntakeBottomRollerSubsystem *bottomRoller,
                         FloorSubsystem *floor, FeederSubsystem *feeder,
                         FlywheelSubsystem *flywheel);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  enum class State { Intaking, Indexing, Reversing };

  IntakeDeployerSubsystem *m_deployer;
  IntakeTopRollerSubsystem *m_topRoller;
  IntakeBottomRollerSubsystem *m_bottomRoller;
  FloorSubsystem *m_floor;
  FeederSubsystem *m_feeder;
  FlywheelSubsystem *m_flywheel;

  State m_state{State::Intaking};
  units::second_t m_stateStartTime{0_s};
  bool m_usingFlywheel{false};
  units::turn_t m_indexStartPosition{0_tr};

  static constexpr units::volt_t kTopRollerVoltage = 10_V;
  static constexpr units::volt_t kBottomRollerVoltage = 10_V;
  static constexpr units::volt_t kFloorIntakeVoltage = 8_V;

  static constexpr units::turn_t kFeederForwardDistance = 20_tr;
  static constexpr units::turn_t kFeederReverseDistance = 4_tr;
  static constexpr units::turn_t kPositionTolerance = 0.25_tr;
  static constexpr units::volt_t kFlywheelReverseVoltage = -1.5_V;
  static constexpr units::volt_t kFeederVoltage = 1.5_V;

  static constexpr units::second_t kIntakeBeforeIndexDuration = 1.5_s;

  static constexpr units::revolutions_per_minute_t kFlywheelIdleThreshold =
      100_rpm;

  bool IsFlywheelIdle() const;
};
