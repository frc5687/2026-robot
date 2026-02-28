// Team 5687 2026

#include "subsystem/flywheel/CTREFlywheelIO.h"

#include <frc/Timer.h>

#include "Constants.h"

using namespace Constants::Flywheel;
using namespace ctre::phoenix6;

CTREFlywheelIO::CTREFlywheelIO(const CANDevice &leader,
                               const CANDevice &follower1,
                               const CANDevice &follower2,
                               const CANDevice &follower3)
    : m_leader(leader.id, leader.bus), m_follower1(follower1.id, follower1.bus),
      m_follower2(follower2.id, follower2.bus),
      m_follower3(follower3.id, follower3.bus),

      m_leaderVelocitySignal(m_leader.GetVelocity()),
      m_leaderPositionSignal(m_leader.GetPosition()),
      m_leaderVoltageSignal(m_leader.GetMotorVoltage()),
      m_leaderStatorCurrentSignal(m_leader.GetStatorCurrent()),
      m_leaderSupplyCurrentSignal(m_leader.GetSupplyCurrent()),

      m_follower1SupplyCurrentSignal(m_follower1.GetSupplyCurrent()),
      m_follower2SupplyCurrentSignal(m_follower2.GetSupplyCurrent()),
      m_follower3SupplyCurrentSignal(m_follower3.GetSupplyCurrent()),

      m_criticalSignals{&m_leaderVelocitySignal, &m_leaderPositionSignal,
                        &m_leaderVoltageSignal, &m_leaderStatorCurrentSignal,
                        &m_leaderSupplyCurrentSignal},
      m_diagnosticSignals{&m_follower1SupplyCurrentSignal,
                          &m_follower2SupplyCurrentSignal,
                          &m_follower3SupplyCurrentSignal} {
  ConfigureDevices();
  ConfigureSignalFrequencies();
}

void CTREFlywheelIO::ConfigureDevices() {
  ConfigureLeader(m_leaderConfig, kLeaderInverted);
  ConfigureClosedLoop();
  m_leader.GetConfigurator().Apply(m_leaderConfig);

  int leaderId = m_leader.GetDeviceID();
  ConfigureFollower(m_follower1, leaderId, kFollower1Opposed);
  ConfigureFollower(m_follower2, leaderId, kFollower2Opposed);
  ConfigureFollower(m_follower3, leaderId, kFollower3Opposed);
}

void CTREFlywheelIO::ConfigureLeader(configs::TalonFXConfiguration &config,
                                     bool inverted) {
  config.MotorOutput.Inverted =
      inverted ? signals::InvertedValue::Clockwise_Positive
               : signals::InvertedValue::CounterClockwise_Positive;
  config.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;

  config.Voltage.PeakForwardVoltage = 12_V;
  config.Voltage.PeakReverseVoltage = -12_V;

  config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
  config.CurrentLimits.StatorCurrentLimitEnable = kEnableStatorCurrent;
  config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
  config.CurrentLimits.SupplyCurrentLimitEnable = kEnableSupplyCurrent;
}

void CTREFlywheelIO::ConfigureFollower(hardware::TalonFX &follower,
                                       int leaderDeviceId, bool opposeLeader) {
  m_followerConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;

  m_followerConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
  m_followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  m_followerConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
  m_followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

  follower.GetConfigurator().Apply(m_followerConfig);
  follower.SetControl(controls::Follower{leaderDeviceId, opposeLeader});
}

void CTREFlywheelIO::ConfigureClosedLoop() {
  m_leaderConfig.Slot0.kS = PID::kS;
  m_leaderConfig.Slot0.kV = PID::kV;
  m_leaderConfig.Slot0.kA = PID::kA;
  m_leaderConfig.Slot0.kP = PID::kP;
  m_leaderConfig.Slot0.kI = PID::kI;
  m_leaderConfig.Slot0.kD = PID::kD;
  m_leaderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
      kClosedLoopRampPeriod;
  m_velocityRequest.UpdateFreqHz = 1000_Hz;
}

void CTREFlywheelIO::ConfigureSignalFrequencies() {
  m_leaderVelocitySignal.SetUpdateFrequency(100_Hz);
  m_leaderPositionSignal.SetUpdateFrequency(100_Hz);
  m_leaderVoltageSignal.SetUpdateFrequency(100_Hz);
  m_leaderStatorCurrentSignal.SetUpdateFrequency(50_Hz);
  m_leaderSupplyCurrentSignal.SetUpdateFrequency(50_Hz);

  m_follower1SupplyCurrentSignal.SetUpdateFrequency(50_Hz);
  m_follower2SupplyCurrentSignal.SetUpdateFrequency(50_Hz);
  m_follower3SupplyCurrentSignal.SetUpdateFrequency(50_Hz);

  m_leader.OptimizeBusUtilization();
  m_follower1.OptimizeBusUtilization();
  m_follower2.OptimizeBusUtilization();
  m_follower3.OptimizeBusUtilization();
}

void CTREFlywheelIO::UpdateInputs(FlywheelIOInputs &inputs) {
  BaseStatusSignal::RefreshAll(m_criticalSignals);
  BaseStatusSignal::RefreshAll(m_diagnosticSignals);

  inputs.leaderMotorPosition = m_leaderPositionSignal.GetValue();
  inputs.leaderMotorVelocity = m_leaderVelocitySignal.GetValue();
  inputs.leaderAppliedVolts = m_leaderVoltageSignal.GetValue();
  inputs.leaderStatorCurrent = m_leaderStatorCurrentSignal.GetValue();
  inputs.leaderSupplyCurrent = m_leaderSupplyCurrentSignal.GetValue();

  inputs.follower1SupplyCurrent = m_follower1SupplyCurrentSignal.GetValue();
  inputs.follower2SupplyCurrent = m_follower2SupplyCurrentSignal.GetValue();
  inputs.follower3SupplyCurrent = m_follower3SupplyCurrentSignal.GetValue();

  inputs.timestamp = units::second_t{frc::Timer::GetFPGATimestamp().value()};

  if (m_characterizing) {
    m_leader.SetControl(m_voltageRequest.WithOutput(m_characterizationVoltage));
  } else if (m_setpoint.value() <= 0) {
    m_leader.SetControl(m_neutralRequest);
  } else {
    m_leader.SetControl(m_velocityRequest.WithVelocity(m_setpoint).WithSlot(0));
  }
}

void CTREFlywheelIO::SetMotorVelocity(units::turns_per_second_t rps) {
  m_characterizing = false;
  m_setpoint = rps;
}

void CTREFlywheelIO::SetVoltage(units::volt_t voltage) {
  m_characterizing = true;
  m_characterizationVoltage = voltage;
}
