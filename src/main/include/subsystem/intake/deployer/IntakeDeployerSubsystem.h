// Team 5687 2026

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/current.h>
#include <units/power.h>
#include <units/time.h>
#include <units/voltage.h>

#include <optional>
#include <memory>

#include <frc/Timer.h>

#include "subsystem/LoggedSubsystem.h"
#include "subsystem/intake/deployer/IntakeDeployerIO.h"

class IntakeDeployerSubsystem : public LoggedSubsystem {
public:
  explicit IntakeDeployerSubsystem(std::unique_ptr<IntakeDeployerIO> io);

  void Deploy();
  void FullyExtend();
  void RetractMid();
  void Retract();

  // Slowly retracts from current position to mid position over the given duration.
  void SlowRetract(units::second_t duration);
  void CancelSlowRetract();
  bool IsSlowRetracting() const;

  void SetPosition(units::meter_t extension);
  void SetVoltage(units::volt_t voltage);
  void ZeroPosition();
  void Stop();

  void SetCurrentLimits(units::ampere_t currentlimit);

  units::meter_t GetPosition() const;
  bool IsDeployed() const;
  bool IsFullyExtended() const;
  bool IsRetracted() const;
  units::ampere_t GetElectricalCurrentDraw() const;
  units::watt_t GetElectricalPowerDraw() const;
  
protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  std::unique_ptr<IntakeDeployerIO> m_io;
  IntakeDeployerIOInputs m_inputs{};

  struct SlowRetractState {
    units::meter_t startPosition;
    units::second_t duration;
    units::second_t startTime;
  };
  std::optional<SlowRetractState> m_slowRetract;

  // Compliant hold: full-force position control that yields to disturbances
  enum class ComplianceState { Holding, Yielding };
  std::optional<units::meter_t> m_compliantTarget;
  ComplianceState m_complianceState{ComplianceState::Holding};


  units::ampere_t m_lastStatorLimit{0_A};

  void DisableCompliantHold();
};
