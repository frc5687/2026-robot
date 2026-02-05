#include "subsystem/intake/CTREIntakeRollerIO.h"

#include "subsystem/intake/IntakeRoller.h"
#include "subsystem/intake/IntakeRollerIO.h"
#include "units/voltage.h"


CTREIntakeRollerIO::CTREIntakeRollerIO(const CANDevice &rightMotor, const CANDevice &leftMotor):
    m_leftMotor(leftMotor.id, leftMotor.bus),
    m_rightMotor(rightMotor.id, rightMotor.bus),
    m_leftVoltage(0_V),
    m_rightVoltage(0_V)
    {
        m_leftConfigs.MotorOutput.Inverted = Constants::IntakeRoller::kLeftMotorInverted ? 
        ctre::phoenix6::signals::InvertedValue::Clockwise_Positive : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
        
        m_rightConfigs.MotorOutput.Inverted = Constants::IntakeRoller::kRightMotorInverted ? 
        ctre::phoenix6::signals::InvertedValue::Clockwise_Positive : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

        m_rightConfigs.CurrentLimits.SupplyCurrentLimit = 20_A;
        m_leftConfigs.CurrentLimits.SupplyCurrentLimit = 20_A;

        m_leftMotor.GetConfigurator().Apply(m_leftConfigs);
        m_rightMotor.GetConfigurator().Apply(m_rightConfigs);
    }

void CTREIntakeRollerIO::UpdateInputs(IntakeRollerIOInputs &inputs){
}

void CTREIntakeRollerIO::SetVoltage(units::volt_t voltage) {
    m_rightMotor.SetControl(m_rightVoltage.WithOutput(voltage));
    m_leftMotor.SetControl(m_leftVoltage.WithOutput(voltage));
}