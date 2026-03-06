// Team 5687 2026

#pragma once

#include <memory>

#include "subsystem/LoggedSubsystem.h"
#include "subsystem/lights/LightsIO.h"

class LightsSubsystem : public LoggedSubsystem {
public:
  explicit LightsSubsystem(std::unique_ptr<LightsIO> io);

protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  std::unique_ptr<LightsIO> m_io;
  LightsIOInputs m_inputs{};
};
