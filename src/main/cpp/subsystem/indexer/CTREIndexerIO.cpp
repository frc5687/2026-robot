// Team 5687 2026

#include "subsystem/indexer/CTREIndexerIO.h"

#include <frc/Timer.h>

#include "Constants.h"

using namespace Constants::Indexer;
using namespace ctre::phoenix6;

CTREIndexerIO::CTREIndexerIO(const CANDevice &leftMotor,
                             const CANDevice &rightMotor,
                             const CANDevice &centerMotor,
                             const CANDevice &centerMotorFollower
                             )
    : m_leftMotor(leftMotor.id, leftMotor.bus),
      m_rightMotor(rightMotor.id, rightMotor.bus),
      m_centerMotor(centerMotor.id, centerMotor.bus),
      m_centerMotorFollower(centerMotorFollower.id, centerMotorFollower.bus),
      m_positionSignal(m_leftMotor.GetPosition()),
      m_velocitySignal(m_leftMotor.GetVelocity()),
      m_voltageSignal(m_leftMotor.GetMotorVoltage()),
      m_statorSignal(m_leftMotor.GetStatorCurrent()),
      m_supplySignal(m_leftMotor.GetSupplyCurrent()),
      m_centerPositionSignal(m_centerMotor.GetPosition()),
      m_centerVelocitySignal(m_centerMotor.GetVelocity()),
      m_centerStatorSignal(m_centerMotor.GetStatorCurrent()),
      m_criticalSignals{&m_positionSignal, &m_velocitySignal, &m_voltageSignal,
                        &m_statorSignal, &m_centerVelocitySignal},
      m_batchedSignals{&m_supplySignal, &m_centerPositionSignal,
                       &m_centerStatorSignal} {
  ConfigureDevices();
  ConfigureSignalFrequencies();
}


void CTREIndexerIO::ConfigureDevices() {
  // ── Leader (left) ────────────────────────────────────────────────────
  m_leftConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
  m_leftConfig.MotorOutput.Inverted =
      kLeftInverted ? signals::InvertedValue::Clockwise_Positive
                    : signals::InvertedValue::CounterClockwise_Positive;

  m_leftConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
  m_leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  m_leftConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
  m_leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

  ConfigureClosedLoop();

  m_leftMotor.GetConfigurator().Apply(m_leftConfig);

  configs::TalonFXConfiguration followerConfig{};
  followerConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
  followerConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
  followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  m_rightMotor.GetConfigurator().Apply(followerConfig);

  m_rightMotor.SetControl(
      controls::Follower{m_leftMotor.GetDeviceID(), kRightOpposed});

  m_centerConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
  m_centerConfig.MotorOutput.Inverted =
      kCenterInverted ? signals::InvertedValue::Clockwise_Positive
                      : signals::InvertedValue::CounterClockwise_Positive;

  m_centerConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
  m_centerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

  m_centerMotor.GetConfigurator().Apply(m_centerConfig);

  m_centerMotorFollower.SetControl(
      controls::Follower{m_centerMotor.GetDeviceID(), kRightOpposed});
}

void CTREIndexerIO::ConfigureClosedLoop() {
  m_leftConfig.Slot0.kS = PID::kS;
  m_leftConfig.Slot0.kV = PID::kV;
  m_leftConfig.Slot0.kA = PID::kA;
  m_leftConfig.Slot0.kP = PID::kP;
  m_leftConfig.Slot0.kI = PID::kI;
  m_leftConfig.Slot0.kD = PID::kD;
}

void CTREIndexerIO::ConfigureSignalFrequencies() {
  m_velocitySignal.SetUpdateFrequency(100_Hz);
  m_positionSignal.SetUpdateFrequency(100_Hz);
  m_voltageSignal.SetUpdateFrequency(100_Hz);
  m_statorSignal.SetUpdateFrequency(100_Hz);
  m_supplySignal.SetUpdateFrequency(50_Hz);

  m_centerVelocitySignal.SetUpdateFrequency(100_Hz);
  m_centerPositionSignal.SetUpdateFrequency(50_Hz);
  m_centerStatorSignal.SetUpdateFrequency(50_Hz);

  m_leftMotor.OptimizeBusUtilization();
  m_rightMotor.OptimizeBusUtilization();
  m_centerMotor.OptimizeBusUtilization();
  m_centerMotorFollower.OptimizeBusUtilization();
}

void CTREIndexerIO::UpdateInputs(IndexerIOInputs &inputs) {
  BaseStatusSignal::RefreshAll(m_criticalSignals);
  BaseStatusSignal::RefreshAll(m_batchedSignals);

  inputs.motorPosition = m_positionSignal.GetValue();
  inputs.motorVelocity = m_velocitySignal.GetValue();
  inputs.centerPosition = m_centerPositionSignal.GetValue();
  inputs.centerVelocity = m_centerVelocitySignal.GetValue();

  inputs.appliedVolts = m_voltageSignal.GetValue();
  inputs.statorCurrent = m_statorSignal.GetValue();
  inputs.supplyCurrent = m_supplySignal.GetValue();
  inputs.centerStatorCurrent = m_centerStatorSignal.GetValue();

  inputs.timestamp = units::second_t{frc::Timer::GetFPGATimestamp().value()};
}


void CTREIndexerIO::SetVoltage(units::volt_t voltage) {
  m_leftMotor.SetControl(m_voltageRequest.WithOutput(voltage));
  m_centerMotor.SetControl(m_voltageRequest.WithOutput(voltage));
}

void CTREIndexerIO::SetMotorVelocity(units::turns_per_second_t rps) {
  m_leftMotor.SetControl(m_velocityRequest.WithVelocity(rps).WithSlot(0));
  m_centerMotor.SetControl(m_velocityRequest.WithVelocity(rps).WithSlot(0));
}

void CTREIndexerIO::Stop() {
  m_leftMotor.SetControl(m_neutralRequest);
  m_centerMotor.SetControl(m_neutralRequest);
}
