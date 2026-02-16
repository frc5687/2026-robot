// Team 5687 2026

#pragma once

#include <memory>

#include "FlywheelIO.h"
#include "FlywheelState.h"
#include "subsystem/LoggedSubsystem.h"

class FlywheelSubsystem : public LoggedSubsystem {
 public:
  explicit FlywheelSubsystem(std::unique_ptr<FlywheelIO> io);
  ~FlywheelSubsystem() = default;
  FlywheelState GetFlywheelState() const;

 protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

 private:
  std::unique_ptr<FlywheelIO> m_io;
  FlywheelIOInputs m_inputs;
};
