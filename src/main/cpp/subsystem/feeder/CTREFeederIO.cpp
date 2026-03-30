// Team 5687 2026

#include "subsystem/feeder/CTREFeederIO.h"

#include <frc/Timer.h>

#include "subsystem/feeder/FeederConstants.h"

using namespace Constants::Feeder;
using namespace ctre::phoenix6;

CTREFeederIO::CTREFeederIO(const CANDevice &leader, const CANDevice &follower)
    : m_leader(leader.id, leader.bus), m_follower(follower.id, follower.bus),
      m_positionSignal(m_leader.GetPosition()),
      m_velocitySignal(m_leader.GetVelocity()),
      m_voltageSignal(m_leader.GetMotorVoltage()),
      m_statorSignal(m_leader.GetStatorCurrent()),
      m_supplySignal(m_leader.GetSupplyCurrent()),
      m_followerStatorSignal(m_follower.GetStatorCurrent()),
      m_followerSupplySignal(m_follower.GetSupplyCurrent()),
      m_followerVoltageSignal(m_follower.GetMotorVoltage()),
      m_criticalSignals{&m_positionSignal,        &m_velocitySignal,
                        &m_voltageSignal,         &m_statorSignal,
                        &m_followerStatorSignal,  &m_followerVoltageSignal},
      m_batchedSignals{&m_supplySignal, &m_followerSupplySignal} {
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

  int leaderId = m_leader.GetDeviceID();
  ConfigureFollower(m_follower, leaderId, kFollowerOpposed);
}

void CTREFeederIO::ConfigureClosedLoop() {
  m_leaderConfig.Slot0.kS = VelocityPID::kS;
  m_leaderConfig.Slot0.kV = VelocityPID::kV;
  m_leaderConfig.Slot0.kA = VelocityPID::kA;
  m_leaderConfig.Slot0.kP = VelocityPID::kP;
  m_leaderConfig.Slot0.kI = VelocityPID::kI;
  m_leaderConfig.Slot0.kD = VelocityPID::kD;

  m_leaderConfig.Slot1.kS = PositionPID::kS;
  m_leaderConfig.Slot1.kV = PositionPID::kV;
  m_leaderConfig.Slot1.kA = PositionPID::kA;
  m_leaderConfig.Slot1.kP = PositionPID::kP;
  m_leaderConfig.Slot1.kI = PositionPID::kI;
  m_leaderConfig.Slot1.kD = PositionPID::kD;
}

void CTREFeederIO::ConfigureFollower(
    ctre::phoenix6::hardware::TalonFX &follower, int leaderDeviceId,
    bool opposeLeader) {
  ctre::phoenix6::configs::TalonFXConfiguration followerConfig{};
  followerConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;

  followerConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
  followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  followerConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
  followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

  follower.GetConfigurator().Apply(followerConfig);
  follower.SetControl(controls::Follower{leaderDeviceId, opposeLeader});
}

void CTREFeederIO::ConfigureSignalFrequencies() {
  m_positionSignal.SetUpdateFrequency(100_Hz);
  m_velocitySignal.SetUpdateFrequency(100_Hz);
  m_voltageSignal.SetUpdateFrequency(100_Hz);
  m_statorSignal.SetUpdateFrequency(100_Hz);
  m_followerStatorSignal.SetUpdateFrequency(100_Hz);
  m_followerVoltageSignal.SetUpdateFrequency(100_Hz);
  m_supplySignal.SetUpdateFrequency(50_Hz);
  m_followerSupplySignal.SetUpdateFrequency(50_Hz);

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
  inputs.followerStatorCurrent = m_followerStatorSignal.GetValue();
  inputs.followerSupplyCurrent = m_followerSupplySignal.GetValue();
  inputs.followerAppliedVolts = m_followerVoltageSignal.GetValue();
  inputs.timestamp = units::second_t{frc::Timer::GetFPGATimestamp().value()};
}

void CTREFeederIO::SetVoltage(units::volt_t voltage) {
  m_leader.SetControl(
      m_voltageRequest.WithOutput(voltage).WithEnableFOC(kEnableFOC));
}

void CTREFeederIO::SetVelocity(units::turns_per_second_t rps) {
  m_leader.SetControl(
      m_velocityRequest.WithVelocity(rps).WithSlot(0).WithEnableFOC(
          kEnableFOC));
}

void CTREFeederIO::SetCurrent(units::ampere_t current) {
  m_leader.SetControl(m_currentRequest.WithOutput(current).WithMaxAbsDutyCycle(1.0));
}

void CTREFeederIO::SetPosition(units::turn_t position) {
  m_leader.SetControl(
      m_positionRequest.WithPosition(position).WithSlot(1).WithEnableFOC(
          kEnableFOC));
}

void CTREFeederIO::Stop() { m_leader.SetControl(m_neutralRequest); }
