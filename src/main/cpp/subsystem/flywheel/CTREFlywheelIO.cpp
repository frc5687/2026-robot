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

CTREFlywheelIO::CTREFlywheelIO(const CANDevice &rightMotorLeader, const CANDevice &rightMotor2, 
  const CANDevice &rightMotor3, const CANDevice &rightMotor4, 
  const CANDevice &leftMotor1, const CANDevice &leftMotor2, const CANDevice &leftMotor3, 
  const CANDevice &leftMotor4) :
  m_rightMotorLeader(rightMotorLeader.id, rightMotorLeader.bus),
  m_rightMotor2(rightMotor2.id, rightMotor2.bus),
  m_rightMotor3(rightMotor3.id, rightMotor3.bus),
  m_rightMotor4(rightMotor4.id, rightMotor4.bus),

  m_leftMotor1(leftMotor1.id, leftMotor1.bus),
  m_leftMotor2(leftMotor2.id, leftMotor2.bus),
  m_leftMotor3(leftMotor3.id, leftMotor3.bus),
  m_leftMotor4(leftMotor4.id, leftMotor4.bus),
  m_request(controls::VelocityTorqueCurrentFOC{0_rpm}.WithSlot(0)),
  m_followerAligned(rightMotorLeader.id, signals::MotorAlignmentValue::Aligned),
  m_followerOpposed(rightMotorLeader.id, signals::MotorAlignmentValue::Opposed),

  m_motorVelocitySignal(m_rightMotorLeader.GetVelocity()),
  m_motorCurrentSignal(m_rightMotorLeader.GetStatorCurrent()),
  m_batchSignals{&m_motorVelocitySignal, &m_motorCurrentSignal}
  {
    m_rightconfig.MotorOutput.Inverted = Constants::Flywheel::kRightMotorInverted ? signals::InvertedValue::Clockwise_Positive : signals::InvertedValue::CounterClockwise_Positive;

    m_rightconfig.Slot0.kP = Constants::Flywheel::kP;
    m_rightconfig.Slot0.kI = Constants::Flywheel::kI;
    m_rightconfig.Slot0.kD = Constants::Flywheel::kD;

    m_rightconfig.Slot0.kS = Constants::Flywheel::kS;
    m_rightconfig.Slot0.kV = Constants::Flywheel::kV;
    m_rightconfig.Slot0.kA = Constants::Flywheel::kA;

    m_rightMotorLeader.GetConfigurator().Apply(m_rightconfig);


    BaseStatusSignal::SetUpdateFrequencyForAll(50_Hz, m_batchSignals);
  }

void CTREFlywheelIO::UpdateInputs(FlywheelIOInputs &inputs){
  BaseStatusSignal::RefreshAll(m_batchSignals);

  inputs.motorVelocity = m_motorVelocitySignal.GetValue();
  inputs.flywheelVelocity = m_motorVelocitySignal.GetValue() * Constants::Flywheel::kGearRatio;

  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void CTREFlywheelIO::SetFlywheelRPM(units::revolutions_per_minute_t desiredVelocity) {
  m_rightMotorLeader.SetControl(m_request.WithVelocity(desiredVelocity / Constants::Flywheel::kGearRatio).WithSlot(0));

  m_leftMotor1.SetControl(m_followerOpposed);
  m_leftMotor2.SetControl(m_followerOpposed);
  m_leftMotor3.SetControl(m_followerOpposed);
  m_leftMotor4.SetControl(m_followerOpposed);

  m_rightMotor2.SetControl(m_followerAligned);
  m_rightMotor3.SetControl(m_followerAligned);
  m_rightMotor4.SetControl(m_followerAligned);

}


