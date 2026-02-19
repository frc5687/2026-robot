// Team 5687 2026

#include "subsystem/intake/CTREIntakeIO.h"

#include <frc/Timer.h>

#include "Constants.h"

using namespace Constants::Intake;
using namespace ctre::phoenix6;

CTREIntakeIO::CTREIntakeIO(const CANDevice &leader, const CANDevice &follower)
    : m_leader(leader.id, leader.bus), m_follower(follower.id, follower.bus),
      m_velocitySignal(m_leader.GetVelocity()),
      m_voltageSignal(m_leader.GetMotorVoltage()),
      m_statorSignal(m_leader.GetStatorCurrent()),
      m_supplySignal(m_leader.GetSupplyCurrent()),
      m_followerStatorSignal(m_follower.GetStatorCurrent()),
      m_criticalSignals{&m_velocitySignal, &m_voltageSignal, &m_statorSignal},
      m_batchedSignals{&m_supplySignal, &m_followerStatorSignal} {
  ConfigureDevices();
}

void CTREIntakeIO::ConfigureDevices() {
  // ── Leader ──────────────────────────────────────────────────────────
  configs::TalonFXConfiguration leaderConfig{};
  leaderConfig.MotorOutput.Inverted =
      kInverted ? signals::InvertedValue::Clockwise_Positive
                : signals::InvertedValue::CounterClockwise_Positive;
  leaderConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;

  leaderConfig.Voltage.PeakForwardVoltage = 12_V;
  leaderConfig.Voltage.PeakReverseVoltage = -12_V;

  leaderConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
  leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  leaderConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
  leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

  m_leader.GetConfigurator().Apply(leaderConfig);

  // ── Follower ────────────────────────────────────────────────────────
  configs::TalonFXConfiguration followerConfig{};
  followerConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
  followerConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
  followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

  m_follower.GetConfigurator().Apply(followerConfig);
  m_follower.SetControl(
      controls::Follower{m_leader.GetDeviceID(), kFollowerOpposed});

  // ── Signal frequencies ──────────────────────────────────────────────
  m_velocitySignal.SetUpdateFrequency(100_Hz);
  m_voltageSignal.SetUpdateFrequency(50_Hz);
  m_statorSignal.SetUpdateFrequency(50_Hz);
  m_supplySignal.SetUpdateFrequency(50_Hz);
  m_followerStatorSignal.SetUpdateFrequency(50_Hz);

  m_leader.OptimizeBusUtilization();
  m_follower.OptimizeBusUtilization();
}

void CTREIntakeIO::UpdateInputs(IntakeIOInputs &inputs) {
  BaseStatusSignal::RefreshAll(m_criticalSignals);
  BaseStatusSignal::RefreshAll(m_batchedSignals);

  inputs.motorVelocity = m_velocitySignal.GetValue();
  inputs.appliedVolts = m_voltageSignal.GetValue();
  inputs.statorCurrent =
      m_statorSignal.GetValue() + m_followerStatorSignal.GetValue();
  inputs.supplyCurrent = m_supplySignal.GetValue();
  inputs.timestamp = units::second_t{frc::Timer::GetFPGATimestamp().value()};
}

void CTREIntakeIO::SetVoltage(units::volt_t voltage) {
  m_leader.SetControl(m_voltageRequest.WithOutput(voltage));
}

void CTREIntakeIO::Stop() { m_leader.SetControl(m_neutralRequest); }
