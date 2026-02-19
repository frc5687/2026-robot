// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include <memory>

#include "subsystem/LoggedSubsystem.h"
#include "subsystem/indexer/IndexerIO.h"
#include "subsystem/indexer/IndexerState.h"

class IndexerSubsystem : public LoggedSubsystem {
public:
  explicit IndexerSubsystem(std::unique_ptr<IndexerIO> io);

  void SetVoltage(units::volt_t voltage);
  void SetVelocity(units::turns_per_second_t rps);
  void Stop();

  bool BallDetected() const { return m_state.ballDetected; }
  bool BallSeated() const { return m_state.ballSeated; }
  bool BallFed() const { return m_state.ballFed; }

  const IndexerState &GetState() const { return m_state; }

protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  std::unique_ptr<IndexerIO> m_io;
  IndexerIOInputs m_inputs{};
  IndexerState m_state{};
};
