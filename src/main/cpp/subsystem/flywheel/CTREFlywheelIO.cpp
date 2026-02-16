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

CTREFlywheelIO::CTREFlywheelIO(const CANDevice &rightLeaderMotor, const CANDevice &rightFollowerMotor, 
  const CANDevice &leftLeaderMotor, const CANDevice &leftFollowerMotor) :
  m_rightLeaderMotor(rightLeaderMotor.id, rightLeaderMotor.bus),
  m_rightFollowerMotor(rightFollowerMotor.id, rightFollowerMotor.bus),
  m_leftLeaderMotor(leftLeaderMotor.id, leftLeaderMotor.bus),
  m_leftFollowerMotor(leftFollowerMotor.id, leftFollowerMotor.bus),

  m_rightLeader(controls::VelocityTorqueCurrentFOC{0_rpm}.WithSlot(0)),
  m_rightFollower(rightLeaderMotor.id, signals::MotorAlignmentValue::Opposed),

  m_leftLeader(controls::VelocityTorqueCurrentFOC{0_rpm}.WithSlot(0)),
  m_leftFollower(leftLeaderMotor.id, signals::MotorAlignmentValue::Opposed),

  m_rightLeaderVelocitySignal(m_rightLeaderMotor.GetVelocity()),
  m_rightLeaderCurrentSignal(m_rightLeaderMotor.GetStatorCurrent()),

  m_leftLeaderVelocitySignal(m_leftLeaderMotor.GetVelocity()),
  m_leftLeaderCurrentSignal(m_leftLeaderMotor.GetStatorCurrent()),

  m_batchSignals{&m_rightLeaderVelocitySignal, &m_rightLeaderCurrentSignal, 
    &m_leftLeaderVelocitySignal, &m_leftLeaderCurrentSignal}
  {
    m_rightLeaderConfig.MotorOutput.Inverted = Constants::Flywheel::kRightMotorInverted ? signals::InvertedValue::Clockwise_Positive : signals::InvertedValue::CounterClockwise_Positive;

    m_rightLeaderConfig.Slot0.kP = Constants::Flywheel::rightkP;
    m_rightLeaderConfig.Slot0.kI = Constants::Flywheel::rightkI;
    m_rightLeaderConfig.Slot0.kD = Constants::Flywheel::rightkD;

    m_rightLeaderConfig.Slot0.kS = Constants::Flywheel::rightkS;
    m_rightLeaderConfig.Slot0.kV = Constants::Flywheel::rightkV;
    m_rightLeaderConfig.Slot0.kA = Constants::Flywheel::rightkA;

    m_rightLeaderMotor.GetConfigurator().Apply(m_rightLeaderConfig);

    m_leftLeaderConfig.MotorOutput.Inverted = Constants::Flywheel::kLeftMotorInverted ? signals::InvertedValue::Clockwise_Positive : signals::InvertedValue::CounterClockwise_Positive;

    m_leftLeaderConfig.Slot0.kP = Constants::Flywheel::leftkP;
    m_leftLeaderConfig.Slot0.kI = Constants::Flywheel::leftkI;
    m_leftLeaderConfig.Slot0.kD = Constants::Flywheel::leftkD;
    
    m_leftLeaderConfig.Slot0.kS = Constants::Flywheel::leftkS;
    m_leftLeaderConfig.Slot0.kV = Constants::Flywheel::leftkV;
    m_leftLeaderConfig.Slot0.kA = Constants::Flywheel::leftkA;

    m_leftLeaderMotor.GetConfigurator().Apply(m_leftLeaderConfig);

    BaseStatusSignal::SetUpdateFrequencyForAll(50_Hz, m_batchSignals);
  }

void CTREFlywheelIO::UpdateInputs(FlywheelIOInputs &inputs){
  BaseStatusSignal::RefreshAll(m_batchSignals);

  inputs.rightFlywheelVelocity = m_rightLeaderVelocitySignal.GetValue() * Constants::Flywheel::kGearRatio;
  inputs.leftFlywheelVelocity = m_rightLeaderVelocitySignal.GetValue() * Constants::Flywheel::kGearRatio;

  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void CTREFlywheelIO::SetFlywheelRPM(units::revolutions_per_minute_t desiredRPMLeft, units::revolutions_per_minute_t desiredRPMRight) {
  m_rightLeaderMotor.SetControl(m_rightLeader.WithVelocity(desiredRPMRight / Constants::Flywheel::kGearRatio).WithSlot(0));
  m_rightFollowerMotor.SetControl(m_rightFollower);

  m_leftLeaderMotor.SetControl(m_leftLeader.WithVelocity(desiredRPMLeft / Constants::Flywheel::kGearRatio).WithSlot(0));
  m_leftFollowerMotor.SetControl(m_leftFollower);
}


