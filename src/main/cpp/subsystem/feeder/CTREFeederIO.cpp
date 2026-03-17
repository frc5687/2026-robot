// Team 5687 2026

#include "subsystem/feeder/CTREFeederIO.h"

#include <frc/Timer.h>

#include "subsystem/feeder/FeederConstants.h"

using namespace Constants::Feeder;
using namespace ctre::phoenix6;

CTREFeederIO::CTREFeederIO(const CANDevice &leader, const CANDevice &follower1,
                           const CANDevice &follower2,
                           const CANDevice &follower3)
    : m_leader(leader.id, leader.bus), m_follower1(follower1.id, follower1.bus),
      m_follower2(follower2.id, follower2.bus),
      m_follower3(follower3.id, follower3.bus),
      m_positionSignal(m_leader.GetPosition()),
      m_velocitySignal(m_leader.GetVelocity()),
      m_voltageSignal(m_leader.GetMotorVoltage()),
      m_statorSignal(m_leader.GetStatorCurrent()),
      m_supplySignal(m_leader.GetSupplyCurrent()),
      m_follower1StatorSignal(m_follower1.GetStatorCurrent()),
      m_follower1SupplySignal(m_follower1.GetSupplyCurrent()),
      m_follower2StatorSignal(m_follower2.GetStatorCurrent()),
      m_follower2SupplySignal(m_follower2.GetSupplyCurrent()),
      m_follower3StatorSignal(m_follower3.GetStatorCurrent()),
      m_follower3SupplySignal(m_follower3.GetSupplyCurrent()),
      m_criticalSignals{&m_positionSignal, &m_velocitySignal, &m_voltageSignal,
                        &m_statorSignal, &m_follower1StatorSignal,
                        &m_follower2StatorSignal, &m_follower3StatorSignal},
      m_batchedSignals{&m_supplySignal, &m_follower1SupplySignal,
                       &m_follower2SupplySignal, &m_follower3SupplySignal} {
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
  ConfigureFollower(m_follower1, leaderId, kFollower1Opposed);
  ConfigureFollower(m_follower2, leaderId, kFollower2Opposed);
  ConfigureFollower(m_follower3, leaderId, kFollower3Opposed);
}

void CTREFeederIO::ConfigureClosedLoop() {
  m_leaderConfig.Slot0.kS = PID::kS;
  m_leaderConfig.Slot0.kV = PID::kV;
  m_leaderConfig.Slot0.kA = PID::kA;
  m_leaderConfig.Slot0.kP = PID::kP;
  m_leaderConfig.Slot0.kI = PID::kI;
  m_leaderConfig.Slot0.kD = PID::kD;
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
  m_follower1StatorSignal.SetUpdateFrequency(100_Hz);
  m_follower2StatorSignal.SetUpdateFrequency(100_Hz);
  m_follower3StatorSignal.SetUpdateFrequency(100_Hz);
  m_supplySignal.SetUpdateFrequency(50_Hz);
  m_follower1SupplySignal.SetUpdateFrequency(50_Hz);
  m_follower2SupplySignal.SetUpdateFrequency(50_Hz);
  m_follower3SupplySignal.SetUpdateFrequency(50_Hz);

  m_leader.OptimizeBusUtilization();
  m_follower1.OptimizeBusUtilization();
  m_follower2.OptimizeBusUtilization();
  m_follower3.OptimizeBusUtilization();
}

void CTREFeederIO::UpdateInputs(FeederIOInputs &inputs) {
  BaseStatusSignal::RefreshAll(m_criticalSignals);
  BaseStatusSignal::RefreshAll(m_batchedSignals);

  inputs.motorPosition = m_positionSignal.GetValue();
  inputs.motorVelocity = m_velocitySignal.GetValue();
  inputs.appliedVolts = m_voltageSignal.GetValue();
  inputs.statorCurrent = m_statorSignal.GetValue();
  inputs.supplyCurrent = m_supplySignal.GetValue();
  inputs.follower1StatorCurrent = m_follower1StatorSignal.GetValue();
  inputs.follower1SupplyCurrent = m_follower1SupplySignal.GetValue();
  inputs.follower2StatorCurrent = m_follower2StatorSignal.GetValue();
  inputs.follower2SupplyCurrent = m_follower2SupplySignal.GetValue();
  inputs.follower3StatorCurrent = m_follower3StatorSignal.GetValue();
  inputs.follower3SupplyCurrent = m_follower3SupplySignal.GetValue();
  inputs.timestamp = units::second_t{frc::Timer::GetFPGATimestamp().value()};
}

void CTREFeederIO::SetVoltage(units::volt_t voltage) {
  m_leader.SetControl(m_voltageRequest.WithOutput(voltage));
}

void CTREFeederIO::SetVelocity(units::turns_per_second_t rps) {
  m_leader.SetControl(m_velocityRequest.WithVelocity(rps).WithSlot(0).WithEnableFOC(false));
}

void CTREFeederIO::Stop() { m_leader.SetControl(m_neutralRequest); }
