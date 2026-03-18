// Team 5687 2026

#include "subsystem/hood/CTREHoodIO.h"

#include <frc/Timer.h>
#include <units/angular_acceleration.h>

#include "subsystem/hood/HoodConstants.h"

using namespace Constants::Hood;
using namespace ctre::phoenix6;

CTREHoodIO::CTREHoodIO(const CANDevice &motor)
    : m_motor(motor.id, motor.bus), m_positionSignal(m_motor.GetPosition()),
      m_velocitySignal(m_motor.GetVelocity()),
      m_voltageSignal(m_motor.GetMotorVoltage()),
      m_statorSignal(m_motor.GetStatorCurrent()),
      m_supplySignal(m_motor.GetSupplyCurrent()),
      m_criticalSignals{&m_positionSignal, &m_velocitySignal, &m_voltageSignal,
                        &m_statorSignal},
      m_batchedSignals{&m_supplySignal} {
  ConfigureDevices();
  ConfigureSignalFrequencies();

  m_motor.SetPosition(0_tr);
}

void CTREHoodIO::ConfigureDevices() {
  m_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
  m_config.MotorOutput.Inverted =
      kInverted ? signals::InvertedValue::Clockwise_Positive
                : signals::InvertedValue::CounterClockwise_Positive;

  m_config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
  m_config.CurrentLimits.StatorCurrentLimitEnable = true;
  m_config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
  m_config.CurrentLimits.SupplyCurrentLimitEnable = true;

  m_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kForwardSoftLimit;
  m_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
  m_config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kReverseSoftLimit;
  m_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

  ConfigureClosedLoop();
  m_motor.GetConfigurator().Apply(m_config);
}

void CTREHoodIO::ConfigureClosedLoop() {
  m_config.Slot0.kS = PID::kS;
  m_config.Slot0.kV = PID::kV;
  m_config.Slot0.kA = PID::kA;
  m_config.Slot0.kP = PID::kP;
  m_config.Slot0.kI = PID::kI;
  m_config.Slot0.kD = PID::kD;

  m_config.MotionMagic.MotionMagicCruiseVelocity =
      units::turns_per_second_t{PID::kCruiseVelocity};
  m_config.MotionMagic.MotionMagicAcceleration =
      units::angular_acceleration::turns_per_second_squared_t{
          PID::kAcceleration};
}

void CTREHoodIO::ConfigureSignalFrequencies() {
  m_positionSignal.SetUpdateFrequency(100_Hz);
  m_velocitySignal.SetUpdateFrequency(100_Hz);
  m_voltageSignal.SetUpdateFrequency(100_Hz);
  m_statorSignal.SetUpdateFrequency(100_Hz);
  m_supplySignal.SetUpdateFrequency(50_Hz);

  m_motor.OptimizeBusUtilization();
}

void CTREHoodIO::UpdateInputs(HoodIOInputs &inputs) {
  BaseStatusSignal::RefreshAll(m_criticalSignals);
  BaseStatusSignal::RefreshAll(m_batchedSignals);

  inputs.motorPosition = m_positionSignal.GetValue();
  inputs.motorVelocity = m_velocitySignal.GetValue();
  inputs.appliedVolts = m_voltageSignal.GetValue();
  inputs.statorCurrent = m_statorSignal.GetValue();
  inputs.supplyCurrent = m_supplySignal.GetValue();
  inputs.timestamp = units::second_t{frc::Timer::GetFPGATimestamp().value()};
}

void CTREHoodIO::SetPosition(units::turn_t position) {
  m_motor.SetControl(m_positionRequest.WithPosition(position).WithSlot(0).WithEnableFOC(
      kEnableFOC));
}

void CTREHoodIO::SetVoltage(units::volt_t voltage) {
  m_motor.SetControl(m_voltageRequest.WithOutput(voltage));
}

void CTREHoodIO::ZeroPosition() { m_motor.SetPosition(0_tr); }

void CTREHoodIO::Stop() { m_motor.SetControl(m_neutralRequest); }
