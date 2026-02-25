// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "subsystem/hood/HoodSubsystem.h"

class ServoCharacterization : public frc2::CommandHelper<frc2::Command, ServoCharacterization> {
public:
  ServoCharacterization(HoodSubsystem *hoodsubsystem);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  HoodSubsystem *m_hoodSubsystem;
  std::vector<std::pair<double, int>> m_leftMicrosecondMap;
  std::vector<std::pair<double, int>> m_rightMicrosecondMap;
  units::second_t m_dt = 0_s;
  units::second_t m_lastMeasurement = 0_s;
  int m_targetMicroseconds = 500;
};
