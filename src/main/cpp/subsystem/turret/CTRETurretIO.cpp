#include "subsystem/turret/CTRETurretIO.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/controls/PositionVoltage.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "subsystem/turret/TurretConstants.h"
#include "subsystem/turret/TurretIO.h"
#include "units/angle.h"
#include "utils/CANDevice.h"
#include <numbers>

using namespace ctre::phoenix6;

CTRETurretIO::CTRETurretIO(const CANDevice &motor) :
  m_motor(motor.id, motor.bus),
  m_controller(controls::PositionVoltage{0_tr}.WithSlot(0)),
  m_motorVelocitySignal(m_motor.GetVelocity()),
  m_motorPositionSignal(m_motor.GetPosition()),
  m_batchSignals{&m_motorVelocitySignal, &m_motorPositionSignal}
  {
    m_config.MotorOutput.Inverted = Constants::Turret::kMotorInverted ? signals::InvertedValue::Clockwise_Positive : signals::InvertedValue::CounterClockwise_Positive;

    m_config.Slot0.kP = Constants::Turret::kP;
    m_config.Slot0.kI = Constants::Turret::kI;
    m_config.Slot0.kD = Constants::Turret::kD;

    m_motor.GetConfigurator().Apply(m_config);

    BaseStatusSignal::SetUpdateFrequencyForAll(50_Hz, m_batchSignals);
  }

void CTRETurretIO::UpdateInputs(TurretIOInputs &inputs) {
  BaseStatusSignal::RefreshAll(m_batchSignals);

  // math might be wrong
  inputs.angle = (m_motorPositionSignal.GetValue() / Constants::Turret::kGearRatio) / (2.0 * std::numbers::pi * 1_rad) * 1_tr;
  inputs.angularVelocity = (m_motorVelocitySignal.GetValue() / Constants::Turret::kGearRatio) / (2.0 * std::numbers::pi * 1_rad_per_s) * 1_tps;
  inputs.motorPosition = m_motorVelocitySignal.GetValue();
  inputs.motorVelocity = m_motorPositionSignal.GetValue();

  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void CTRETurretIO::SetTurretAngle(units::radian_t angle) {
  m_motor.SetControl(m_controller.WithPosition(angle));
}
