// Team 5687 2026

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/voltage.h>

#include <memory>

#include "subsystem/LoggedSubsystem.h"
#include "subsystem/intake/deployer/IntakeDeployerIO.h"

class IntakeDeployerSubsystem : public LoggedSubsystem {
public:
  explicit IntakeDeployerSubsystem(std::unique_ptr<IntakeDeployerIO> io);

  void Deploy();
  void RetractMid();
  void Retract();

  // Not tested at all
  void SetPosition(units::meter_t extension);
  void SetVoltage(units::volt_t voltage);
  void ZeroPosition();
  void Stop();

  units::meter_t GetPosition() const;
  bool IsDeployed() const;
  bool IsRetracted() const;

protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  std::unique_ptr<IntakeDeployerIO> m_io;
  IntakeDeployerIOInputs m_inputs{};
};
