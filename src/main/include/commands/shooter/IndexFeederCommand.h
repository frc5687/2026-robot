// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/voltage.h>

#include "subsystem/feeder/FeederSubsystem.h"

class IndexFeederCommand
    : public frc2::CommandHelper<frc2::Command, IndexFeederCommand> {
public:
  IndexFeederCommand(FeederSubsystem *feeder);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  enum class State { Feeding, JogForward, JogBackward, Done };

  FeederSubsystem *m_feeder;

  State m_state{State::Feeding};
  int m_currentCycle{0};
  units::turn_t m_jogStartPosition{0_tr};

  static constexpr int kIndexCycles = 3;
  static constexpr units::volt_t kFeederFeedVoltage = 2_V;
  static constexpr units::turn_t kJogForwardRotations = 3_tr;
  static constexpr units::turn_t kJogBackwardRotations = 2_tr;
  static constexpr units::turn_t kFinalJogBackwardRotations = 4_tr;
};
