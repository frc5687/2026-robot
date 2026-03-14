// Team 5687 2026

#include "subsystem/intake/bottomroller/CTREIntakeBottomRollerIO.h"

#include <frc/Timer.h>

#include "Constants.h"

using namespace Constants::IntakeBottomRoller;
using namespace ctre::phoenix6;

CTREIntakeBottomRollerIO::CTREIntakeBottomRollerIO(const CANDevice &motor)
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
}

void CTREIntakeBottomRollerIO::ConfigureDevices() {
  m_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
  m_config.MotorOutput.Inverted =
      kInverted ? signals::InvertedValue::Clockwise_Positive
                : signals::InvertedValue::CounterClockwise_Positive;

  m_config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
  m_config.CurrentLimits.StatorCurrentLimitEnable = true;
  m_config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
  m_config.CurrentLimits.SupplyCurrentLimitEnable = true;

  ConfigureClosedLoop();
  m_motor.GetConfigurator().Apply(m_config);
}

void CTREIntakeBottomRollerIO::ConfigureClosedLoop() {
  m_config.Slot0.kS = PID::kS;
  m_config.Slot0.kV = PID::kV;
  m_config.Slot0.kA = PID::kA;
  m_config.Slot0.kP = PID::kP;
  m_config.Slot0.kI = PID::kI;
  m_config.Slot0.kD = PID::kD;
}

void CTREIntakeBottomRollerIO::ConfigureSignalFrequencies() {
  m_positionSignal.SetUpdateFrequency(100_Hz);
  m_velocitySignal.SetUpdateFrequency(100_Hz);
  m_voltageSignal.SetUpdateFrequency(100_Hz);
  m_statorSignal.SetUpdateFrequency(100_Hz);
  m_supplySignal.SetUpdateFrequency(50_Hz);

  m_motor.OptimizeBusUtilization();
}

void CTREIntakeBottomRollerIO::UpdateInputs(
    IntakeBottomRollerIOInputs &inputs) {
  BaseStatusSignal::RefreshAll(m_criticalSignals);
  BaseStatusSignal::RefreshAll(m_batchedSignals);

  inputs.motorPosition = m_positionSignal.GetValue();
  inputs.motorVelocity = m_velocitySignal.GetValue();
  inputs.appliedVolts = m_voltageSignal.GetValue();
  inputs.statorCurrent = m_statorSignal.GetValue();
  inputs.supplyCurrent = m_supplySignal.GetValue();
  inputs.timestamp = units::second_t{frc::Timer::GetFPGATimestamp().value()};
}

void CTREIntakeBottomRollerIO::SetVoltage(units::volt_t voltage) {
  m_motor.SetControl(m_voltageRequest.WithOutput(voltage).WithEnableFOC(false));
}

void CTREIntakeBottomRollerIO::SetVelocity(units::turns_per_second_t rps) {
  m_motor.SetControl(m_velocityRequest.WithVelocity(rps).WithSlot(0).WithEnableFOC(false));
}

void CTREIntakeBottomRollerIO::Stop() { m_motor.SetControl(m_neutralRequest); }
