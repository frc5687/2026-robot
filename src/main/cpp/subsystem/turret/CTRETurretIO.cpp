// Team 5687 2026

#include "subsystem/turret/CTRETurretIO.h"

#include <frc/Timer.h>

#include "Constants.h"
#include "utils/CANDevice.h"

using namespace Constants::Turret;
using namespace ctre::phoenix6;

CTRETurretIO::CTRETurretIO(const CANDevice &ids, int hallEffectId)
    : m_motor(ids.id, ids.bus), m_hallEffect(hallEffectId),
      m_positionSignal(m_motor.GetPosition()),
      m_velocitySignal(m_motor.GetVelocity()),
      m_accelerationSignal(m_motor.GetAcceleration()),
      m_currentSignal(m_motor.GetSupplyCurrent()),
      m_torqueCurrentSignal(m_motor.GetTorqueCurrent()),
      m_tempSignal(m_motor.GetDeviceTemp()) {
  ConfigureDevices();
  ConfigureSignalFrequencies();
}

void CTRETurretIO::ConfigureDevices() {
  m_motorConfig.MotorOutput.Inverted =
      kMotorInverted ? signals::InvertedValue::Clockwise_Positive
                     : signals::InvertedValue::CounterClockwise_Positive;
  m_motorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

  m_motorConfig.Voltage.PeakForwardVoltage = 12_V;
  m_motorConfig.Voltage.PeakReverseVoltage = -12_V;

  m_motorConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
  m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  m_motorConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
  m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

  const units::turn_t forwardLimit =
      units::turn_t{kMaxAngle / (2.0 * std::numbers::pi * 1_rad) * 1_tr} *
      kGearRatio;
  const units::turn_t reverseLimit =
      units::turn_t{kMinAngle / (2.0 * std::numbers::pi * 1_rad) * 1_tr} *
      kGearRatio;

  m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
  m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
  m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;

  ConfigureClosedLoop();

  m_motor.GetConfigurator().Apply(m_motorConfig);
}

void CTRETurretIO::ConfigureClosedLoop() {
  m_motorConfig.Slot0.kP = PID::kP;
  m_motorConfig.Slot0.kI = PID::kI;
  m_motorConfig.Slot0.kD = PID::kD;
  m_motorConfig.Slot0.kS = PID::kS;
  m_motorConfig.Slot0.kV = PID::kV;
  m_motorConfig.Slot0.kA = PID::kA;

  m_motorConfig.MotionMagic.MotionMagicCruiseVelocity = kCruiseVelocity;
  m_motorConfig.MotionMagic.MotionMagicAcceleration = kAcceleration;
  m_motorConfig.MotionMagic.MotionMagicJerk = kJerk;
}

void CTRETurretIO::ConfigureSignalFrequencies() {
  m_positionSignal.SetUpdateFrequency(100_Hz);
  m_velocitySignal.SetUpdateFrequency(100_Hz);
  m_accelerationSignal.SetUpdateFrequency(100_Hz);
  m_currentSignal.SetUpdateFrequency(50_Hz);
  m_torqueCurrentSignal.SetUpdateFrequency(50_Hz);
  m_tempSignal.SetUpdateFrequency(4_Hz);

  m_motor.OptimizeBusUtilization();
}

void CTRETurretIO::UpdateInputs(TurretIOInputs &inputs) {
  BaseStatusSignal::RefreshAll(m_positionSignal, m_velocitySignal,
                               m_accelerationSignal, m_currentSignal,
                               m_torqueCurrentSignal, m_tempSignal);

  // Raw rotor turns — TurretSubsystem applies gear ratio
  inputs.motorPosition = BaseStatusSignal::GetLatencyCompensatedValue(
      m_positionSignal, m_velocitySignal);
  inputs.motorVelocity = m_velocitySignal.GetValue();
  inputs.motorAcceleration = m_accelerationSignal.GetValue();
  inputs.motorCurrent = m_currentSignal.GetValue();
  inputs.motorTorque = kMotor.Kt * m_torqueCurrentSignal.GetValue();
  inputs.timestamp = frc::Timer::GetFPGATimestamp();
  inputs.hallEffectTriggered = m_hallEffect.Get();
}

void CTRETurretIO::SetTurretAngle(units::radian_t angle) {
  // Subsystem passes the desired mechanism angle.
  // Convert: mechanism radians → mechanism turns → rotor turns
  const units::turn_t mechanismTurns{angle / (2.0 * std::numbers::pi * 1_rad) *
                                     1_tr};
  const units::turn_t rotorTurns = mechanismTurns * kGearRatio;

  m_motor.SetControl(m_positionRequest.WithPosition(rotorTurns).WithSlot(0));
}

void CTRETurretIO::Stop() {
  m_motor.SetControl(m_voltageRequest.WithOutput(0_V));
}

void CTRETurretIO::SetBrakeMode(bool brake) {
  m_motor.GetConfigurator().Refresh(m_motorConfig);
  m_motorConfig.MotorOutput.NeutralMode =
      brake ? signals::NeutralModeValue::Brake
            : signals::NeutralModeValue::Coast;
  m_motor.GetConfigurator().Apply(m_motorConfig);
}

void CTRETurretIO::ZeroPosition() { m_motor.SetPosition(0_tr); }

void CTRETurretIO::ResetEncoderAngle(units::radian_t angle) {
  // Subsystem passes the desired mechanism angle.
  // Convert: mechanism radians → mechanism turns → rotor turns
  const units::turn_t mechanismTurns{angle / (2.0 * std::numbers::pi * 1_rad) *
                                     1_tr};
  const units::turn_t rotorTurns = mechanismTurns * kGearRatio;

  m_motor.SetPosition(rotorTurns);
}
