#include "subsystem/flywheel/CTREFlywheelIO.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/controls/VelocityVoltage.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "frc/Timer.h"
#include "subsystem/flywheel/FlywheelIO.h"
#include "units/angular_velocity.h"
#include "utils/CANDevice.h"
#include "subsystem/flywheel/FlywheelConstants.h"

using namespace ctre::phoenix6;

CTREFlywheelIO::CTREFlywheelIO(const CANDevice &motor) :
  m_motor(motor.id, motor.bus),
  m_request(controls::VelocityVoltage{0_rpm}.WithSlot(0)),
  m_motorVelocitySignal(m_motor.GetVelocity())
  {
    m_config.MotorOutput.Inverted = Constants::Flywheel::kMotorInverted ? signals::InvertedValue::Clockwise_Positive : signals::InvertedValue::CounterClockwise_Positive;

    m_config.Slot0.kP = Constants::Flywheel::kP;
    m_config.Slot0.kI = Constants::Flywheel::kI;
    m_config.Slot0.kD = Constants::Flywheel::kD;

    m_config.Slot0.kS = Constants::Flywheel::kS;
    m_config.Slot0.kV = Constants::Flywheel::kV;
    m_config.Slot0.kA = Constants::Flywheel::kA;

    m_motor.GetConfigurator().Apply(m_config);
  };

void CTREFlywheelIO::UpdateInputs(FlywheelIOInputs &inputs){
  inputs.motorVelocity = m_motorVelocitySignal.GetValue();
  inputs.flywheelVelocity = m_motorVelocitySignal.GetValue() / Constants::Flywheel::kGearRatio;

  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void CTREFlywheelIO::SetFlywheelRPM(units::revolutions_per_minute_t desiredVelocity) {
  m_motor.SetControl(m_request.WithVelocity(desiredVelocity));
}


