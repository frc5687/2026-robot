#include "subsystem/flywheel/CTREFlywheelIO.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/VelocityVoltage.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "frc/Timer.h"
#include "subsystem/flywheel/FlywheelIO.h"
#include "units/angular_velocity.h"
#include "utils/CANDevice.h"
#include "subsystem/flywheel/FlywheelConstants.h"
#include "units/frequency.h"

using namespace ctre::phoenix6;

CTREFlywheelIO::CTREFlywheelIO(const CANDevice &rightMotor, const CANDevice &leftMotor) :
  m_rightMotor(rightMotor.id, rightMotor.bus),
  m_leftMotor(leftMotor.id, leftMotor.bus),
  m_request(controls::VelocityTorqueCurrentFOC{0_rpm}.WithSlot(0)),
  m_follower(rightMotor.id, signals::MotorAlignmentValue::Opposed),
  m_motorVelocitySignal(m_leftMotor.GetVelocity()),
  m_motorCurrentSignal(m_leftMotor.GetStatorCurrent()),
  m_batchSignals{&m_motorVelocitySignal, &m_motorCurrentSignal}
  {
    m_rightconfig.MotorOutput.Inverted = Constants::Flywheel::kRightMotorInverted ? signals::InvertedValue::Clockwise_Positive : signals::InvertedValue::CounterClockwise_Positive;

    m_rightconfig.Slot0.kP = Constants::Flywheel::kP;
    m_rightconfig.Slot0.kI = Constants::Flywheel::kI;
    m_rightconfig.Slot0.kD = Constants::Flywheel::kD;

    m_rightconfig.Slot0.kS = Constants::Flywheel::kS;
    m_rightconfig.Slot0.kV = Constants::Flywheel::kV;
    m_rightconfig.Slot0.kA = Constants::Flywheel::kA;

    m_rightMotor.GetConfigurator().Apply(m_rightconfig);


    BaseStatusSignal::SetUpdateFrequencyForAll(50_Hz, m_batchSignals);
  }

void CTREFlywheelIO::UpdateInputs(FlywheelIOInputs &inputs){
  BaseStatusSignal::RefreshAll(m_batchSignals);

  inputs.motorVelocity = m_motorVelocitySignal.GetValue();
  inputs.flywheelVelocity = m_motorVelocitySignal.GetValue() * Constants::Flywheel::kGearRatio;

  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void CTREFlywheelIO::SetFlywheelRPM(units::revolutions_per_minute_t desiredVelocity) {
  m_leftMotor.SetControl(m_follower);
  m_rightMotor.SetControl(m_request.WithVelocity(desiredVelocity / Constants::Flywheel::kGearRatio).WithSlot(0));

}


