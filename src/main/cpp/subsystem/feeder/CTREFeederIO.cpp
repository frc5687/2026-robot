// Team 5687 2026

#include "subsystem/feeder/CTREFeederIO.h"

#include <frc/Timer.h>

#include "Constants.h"

using namespace Constants::Feeder;
using namespace ctre::phoenix6;

CTREFeederIO::CTREFeederIO(const CANDevice &leader, const CANDevice &follower)
    : m_leader(leader.id, leader.bus), m_follower(follower.id, follower.bus),
      m_positionSignal(m_leader.GetPosition()),
      m_velocitySignal(m_leader.GetVelocity()),
      m_voltageSignal(m_leader.GetMotorVoltage()),
      m_statorSignal(m_leader.GetStatorCurrent()),
      m_supplySignal(m_leader.GetSupplyCurrent()),
      m_criticalSignals{&m_positionSignal, &m_velocitySignal, &m_voltageSignal,
                        &m_statorSignal},
      m_batchedSignals{&m_supplySignal} {
  ConfigureDevices();
  ConfigureSignalFrequencies();
}

void CTREFeederIO::ConfigureDevices() {
  m_leaderConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
  m_leaderConfig.MotorOutput.Inverted =
      kLeaderInverted ? signals::InvertedValue::Clockwise_Positive
                      : signals::InvertedValue::CounterClockwise_Positive;

  m_leaderConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
  m_leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  m_leaderConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
  m_leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

  ConfigureClosedLoop();
  m_leader.GetConfigurator().Apply(m_leaderConfig);

  configs::TalonFXConfiguration followerConfig{};
  followerConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
  followerConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
  followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  m_follower.GetConfigurator().Apply(followerConfig);

  m_follower.SetControl(
      controls::Follower{m_leader.GetDeviceID(), kFollowerOpposed});
}

void CTREFeederIO::ConfigureClosedLoop() {
  m_leaderConfig.Slot0.kS = PID::kS;
  m_leaderConfig.Slot0.kV = PID::kV;
  m_leaderConfig.Slot0.kA = PID::kA;
  m_leaderConfig.Slot0.kP = PID::kP;
  m_leaderConfig.Slot0.kI = PID::kI;
  m_leaderConfig.Slot0.kD = PID::kD;
}

void CTREFeederIO::ConfigureSignalFrequencies() {
  m_positionSignal.SetUpdateFrequency(100_Hz);
  m_velocitySignal.SetUpdateFrequency(100_Hz);
  m_voltageSignal.SetUpdateFrequency(100_Hz);
  m_statorSignal.SetUpdateFrequency(100_Hz);
  m_supplySignal.SetUpdateFrequency(50_Hz);

  m_leader.OptimizeBusUtilization();
  m_follower.OptimizeBusUtilization();
}

void CTREFeederIO::UpdateInputs(FeederIOInputs &inputs) {
  BaseStatusSignal::RefreshAll(m_criticalSignals);
  BaseStatusSignal::RefreshAll(m_batchedSignals);

  inputs.motorPosition = m_positionSignal.GetValue();
  inputs.motorVelocity = m_velocitySignal.GetValue();
  inputs.appliedVolts = m_voltageSignal.GetValue();
  inputs.statorCurrent = m_statorSignal.GetValue();
  inputs.supplyCurrent = m_supplySignal.GetValue();
  inputs.timestamp = units::second_t{frc::Timer::GetFPGATimestamp().value()};
}

void CTREFeederIO::SetVoltage(units::volt_t voltage) {
  m_leader.SetControl(m_voltageRequest.WithOutput(voltage).WithEnableFOC(false));
}

void CTREFeederIO::SetVelocity(units::turns_per_second_t rps) {
  m_leader.SetControl(m_velocityRequest.WithVelocity(rps).WithSlot(0).WithEnableFOC(false));
}

void CTREFeederIO::Stop() { m_leader.SetControl(m_neutralRequest); }
