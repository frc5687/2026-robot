// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/time.h>
#include <units/voltage.h>

#include "subsystem/feeder/FeederSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"

class IndexFeederCommand
    : public frc2::CommandHelper<frc2::Command, IndexFeederCommand> {
public:
  IndexFeederCommand(FeederSubsystem *feeder, FlywheelSubsystem *flywheel);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  enum class State { Indexing, Reversing, Done };

  FeederSubsystem *m_feeder;
  FlywheelSubsystem *m_flywheel;

  State m_state{State::Indexing};
  units::second_t m_stateStartTime{0_s};

  static constexpr units::volt_t kFeederIndexVoltage = 3_V;
  static constexpr units::volt_t kFlywheelReverseVoltage = -1.5_V;
  static constexpr units::volt_t kFeederReverseVoltage = -2_V;
  static constexpr units::second_t kIndexDuration = 0.65_s;
  static constexpr units::second_t kReverseDuration = 0.15_s;
};
