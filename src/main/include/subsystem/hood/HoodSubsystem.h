// Team 5687 2026

#pragma once

#include <units/angle.h>
#include <units/current.h>
#include <units/power.h>
#include <units/voltage.h>

#include <memory>

#include "subsystem/LoggedSubsystem.h"
#include "subsystem/hood/HoodIO.h"
#include "subsystem/hood/HoodState.h"

class HoodSubsystem : public LoggedSubsystem {
public:
  explicit HoodSubsystem(std::unique_ptr<HoodIO> io);

  void SetPosition(units::radian_t mechanismAngle);
  void SetVoltage(units::volt_t voltage);
  void ZeroPosition();
  void Stop();

  units::radian_t GetPosition() const;
  bool IsAtPosition(units::radian_t target) const;

  HoodState GetHoodState() const;
  units::ampere_t GetElectricalCurrentDraw() const;
  units::watt_t GetElectricalPowerDraw() const;

protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  std::unique_ptr<HoodIO> m_io;
  HoodIOInputs m_inputs{};
};
