// Team 5687 2026

#include "subsystem/intake/toproller/CTREIntakeTopRollerIO.h"

#include <frc/Timer.h>

#include "subsystem/intake/IntakeConstants.h"

using namespace Constants::IntakeTopRoller;
using namespace ctre::phoenix6;

CTREIntakeTopRollerIO::CTREIntakeTopRollerIO(const CANDevice &leader,
                                             const CANDevice &follower)
    : m_leader(leader.id, leader.bus), m_follower(follower.id, follower.bus),
      m_positionSignal(m_leader.GetPosition()),
      m_velocitySignal(m_leader.GetVelocity()),
      m_voltageSignal(m_leader.GetMotorVoltage()),
      m_statorSignal(m_leader.GetStatorCurrent()),
      m_supplySignal(m_leader.GetSupplyCurrent()),
      m_followerStatorSignal(m_follower.GetStatorCurrent()),
      m_followerSupplySignal(m_follower.GetSupplyCurrent()),
      m_followerVoltageSignal(m_follower.GetMotorVoltage()),
      m_criticalSignals{&m_positionSignal,       &m_velocitySignal,
                        &m_voltageSignal,        &m_statorSignal,
                        &m_followerStatorSignal, &m_followerVoltageSignal},
      m_batchedSignals{&m_supplySignal, &m_followerSupplySignal} {
  ConfigureDevices();
  ConfigureSignalFrequencies();
}

void CTREIntakeTopRollerIO::ConfigureDevices() {
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
  followerConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
  followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  m_follower.GetConfigurator().Apply(followerConfig);

  m_follower.SetControl(
      controls::Follower{m_leader.GetDeviceID(), kFollowerOpposed});
}

void CTREIntakeTopRollerIO::ConfigureClosedLoop() {
  m_leaderConfig.Slot0.kS = PID::kS;
  m_leaderConfig.Slot0.kV = PID::kV;
  m_leaderConfig.Slot0.kA = PID::kA;
  m_leaderConfig.Slot0.kP = PID::kP;
  m_leaderConfig.Slot0.kI = PID::kI;
  m_leaderConfig.Slot0.kD = PID::kD;
}

void CTREIntakeTopRollerIO::ConfigureSignalFrequencies() {
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

void CTREIntakeTopRollerIO::UpdateInputs(IntakeTopRollerIOInputs &inputs) {
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

void CTREIntakeTopRollerIO::SetVoltage(units::volt_t voltage) {
  m_leader.SetControl(
      m_voltageRequest.WithOutput(voltage).WithEnableFOC(kEnableFOC));
}

void CTREIntakeTopRollerIO::SetVelocity(units::turns_per_second_t rps) {
  m_leader.SetControl(
      m_velocityRequest.WithVelocity(rps).WithSlot(0).WithEnableFOC(
          kEnableFOC));
}

void CTREIntakeTopRollerIO::Stop() { m_leader.SetControl(m_neutralRequest); }
