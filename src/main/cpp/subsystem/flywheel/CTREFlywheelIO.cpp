// Team 5687 2026

#include "subsystem/flywheel/CTREFlywheelIO.h"

#include <frc/Timer.h>

#include "Constants.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"

using namespace Constants::Flywheel;
using namespace ctre::phoenix6;
CTREFlywheelIO::CTREFlywheelIO(const CANDevice &leftLeader,
                               const CANDevice &leftFollower,
                               const CANDevice &rightLeader,
                               const CANDevice &rightFollower)
    : m_leftLeader(leftLeader.id, leftLeader.bus),
      m_leftFollower(leftFollower.id, leftFollower.bus),
      m_rightLeader(rightLeader.id, rightLeader.bus),
      m_rightFollower(rightFollower.id, rightFollower.bus),

      m_leftVelocitySignal(m_leftLeader.GetVelocity()),
      m_leftPositionSignal(m_leftLeader.GetPosition()),
      m_leftVoltageSignal(m_leftLeader.GetMotorVoltage()),
      m_leftStatorCurrentSignal(m_leftLeader.GetStatorCurrent()),
      m_leftSupplyCurrentSignal(m_leftLeader.GetSupplyCurrent()),
      m_rightVelocitySignal(m_rightLeader.GetVelocity()),
      m_rightPositionSignal(m_rightLeader.GetPosition()),
      m_rightVoltageSignal(m_rightLeader.GetMotorVoltage()),
      m_rightStatorCurrentSignal(m_rightLeader.GetStatorCurrent()),
      m_rightSupplyCurrentSignal(m_rightLeader.GetSupplyCurrent()),
      m_leftFollowerStatorCurrentSignal(m_leftFollower.GetStatorCurrent()),
      m_rightFollowerStatorCurrentSignal(m_rightFollower.GetStatorCurrent()),
      m_criticalSignals{&m_leftVelocitySignal,  &m_leftPositionSignal,
                        &m_leftVoltageSignal,   &m_rightVelocitySignal,
                        &m_rightPositionSignal, &m_rightVoltageSignal},
      m_batchedSignals{&m_leftStatorCurrentSignal,
                       &m_leftSupplyCurrentSignal,
                       &m_rightStatorCurrentSignal,
                       &m_rightSupplyCurrentSignal,
                       &m_leftFollowerStatorCurrentSignal,
                       &m_rightFollowerStatorCurrentSignal} {
  ConfigureDevices();
  ConfigureSignalFrequencies();
}

void CTREFlywheelIO::ConfigureDevices() {
  ConfigureLeader(m_leftLeaderConfig, kLeftInverted);
  ConfigureLeader(m_rightLeaderConfig, kRightInverted);
  ConfigureClosedLoop();

  m_leftLeader.GetConfigurator().Apply(m_leftLeaderConfig);
  m_rightLeader.GetConfigurator().Apply(m_rightLeaderConfig);

  ConfigureFollower(m_leftFollower, m_leftLeader.GetDeviceID(),
                    kLeftFollowerOpposed);
  ConfigureFollower(m_rightFollower, m_rightLeader.GetDeviceID(),
                    kRightFollowerOpposed);
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
  m_leftLeaderConfig.Slot0.kS = PID::kS;
  m_leftLeaderConfig.Slot0.kV = PID::kV;
  m_leftLeaderConfig.Slot0.kA = PID::kA;
  m_leftLeaderConfig.Slot0.kP = PID::kP;
  m_leftLeaderConfig.Slot0.kI = PID::kI;
  m_leftLeaderConfig.Slot0.kD = PID::kD;

  m_rightLeaderConfig.Slot0.kS = PID::kS;
  m_rightLeaderConfig.Slot0.kV = PID::kV;
  m_rightLeaderConfig.Slot0.kA = PID::kA;
  m_rightLeaderConfig.Slot0.kP = PID::kP;
  m_rightLeaderConfig.Slot0.kI = PID::kI;
  m_rightLeaderConfig.Slot0.kD = PID::kD;
}

void CTREFlywheelIO::ConfigureSignalFrequencies() {
  m_leftVelocitySignal.SetUpdateFrequency(100_Hz);
  m_leftPositionSignal.SetUpdateFrequency(100_Hz);
  m_leftVoltageSignal.SetUpdateFrequency(100_Hz);
  m_leftStatorCurrentSignal.SetUpdateFrequency(50_Hz);
  m_leftSupplyCurrentSignal.SetUpdateFrequency(50_Hz);

  m_rightVelocitySignal.SetUpdateFrequency(100_Hz);
  m_rightPositionSignal.SetUpdateFrequency(100_Hz);
  m_rightVoltageSignal.SetUpdateFrequency(100_Hz);
  m_rightStatorCurrentSignal.SetUpdateFrequency(50_Hz);
  m_rightSupplyCurrentSignal.SetUpdateFrequency(50_Hz);

  m_leftFollowerStatorCurrentSignal.SetUpdateFrequency(50_Hz);
  m_rightFollowerStatorCurrentSignal.SetUpdateFrequency(50_Hz);

  m_leftLeader.OptimizeBusUtilization();
  m_leftFollower.OptimizeBusUtilization();
  m_rightLeader.OptimizeBusUtilization();
  m_rightFollower.OptimizeBusUtilization();
}

void CTREFlywheelIO::UpdateInputs(FlywheelIOInputs &inputs) {
  BaseStatusSignal::RefreshAll(m_criticalSignals);
  BaseStatusSignal::RefreshAll(m_batchedSignals);

  inputs.leftMotorPosition = m_leftPositionSignal.GetValue();
  inputs.rightMotorPosition = m_rightPositionSignal.GetValue();

  inputs.leftMotorVelocity = m_leftVelocitySignal.GetValue();
  inputs.rightMotorVelocity = m_rightVelocitySignal.GetValue();

  inputs.leftAppliedVolts = m_leftVoltageSignal.GetValue();
  inputs.rightAppliedVolts = m_rightVoltageSignal.GetValue();

  inputs.leftStatorCurrent = m_leftStatorCurrentSignal.GetValue() +
                             m_leftFollowerStatorCurrentSignal.GetValue();
  inputs.rightStatorCurrent = m_rightStatorCurrentSignal.GetValue() +
                              m_rightFollowerStatorCurrentSignal.GetValue();

  inputs.leftSupplyCurrent = m_leftSupplyCurrentSignal.GetValue();
  inputs.rightSupplyCurrent = m_rightSupplyCurrentSignal.GetValue();

  inputs.timestamp = units::second_t{frc::Timer::GetFPGATimestamp().value()};

  if (m_characterizing) {
    m_leftLeader.SetControl(
        m_voltageRequest.WithOutput(m_characterizationVoltage));
    m_rightLeader.SetControl(
        m_voltageRequest.WithOutput(m_characterizationVoltage));
  } else if (m_leftSetpoint.value() <= 0 && m_rightSetpoint.value() <= 0) {
    m_leftLeader.SetControl(m_neutralRequest);
    m_rightLeader.SetControl(m_neutralRequest);
  } else {
    m_leftLeader.SetControl(
        m_velocityRequest.WithVelocity(m_leftSetpoint).WithSlot(0));
    m_rightLeader.SetControl(
        m_velocityRequest.WithVelocity(m_rightSetpoint).WithSlot(0));
  }
}

void CTREFlywheelIO::SetMotorVelocity(units::turns_per_second_t leftRPS,
                                      units::turns_per_second_t rightRPS) {
  m_characterizing = false;
  m_leftSetpoint = leftRPS;
  m_rightSetpoint = rightRPS;
}

void CTREFlywheelIO::SetVoltage(units::volt_t voltage) {
  m_characterizing = true;
  m_characterizationVoltage = voltage;
}
