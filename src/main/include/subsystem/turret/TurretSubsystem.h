// Team 5687 2026

#pragma once

#include <memory>

#include "TurretIO.h"
#include "TurretState.h"
#include "subsystem/LoggedSubsystem.h"

class TurretSubsystem : public LoggedSubsystem {
 public:
  explicit TurretSubsystem(std::unique_ptr<TurretIO> io);
  ~TurretSubsystem() = default;

  void SetAngle(units::radian_t desiredAngle);
  TurretState GetTurretState();

 protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

 private:
  std::unique_ptr<TurretIO> m_io;
  TurretIOInputs m_inputs{};

  units::radian_t m_desiredAngle{0_rad};
};
