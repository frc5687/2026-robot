#include "subsystem/indexer/CTREIndexerIO.h"

#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "subsystem/indexer/IndexerConstants.h"
#include "subsystem/indexer/IndexerIO.h"
#include "units/angular_velocity.h"
#include "units/voltage.h"
#include "utils/CANDevice.h"


CTREIndexerIO::CTREIndexerIO(const CANDevice &rightIndexer, const CANDevice &leftIndexer, const CANDevice &centerIndexer):
    m_leftMotor(leftIndexer.id, leftIndexer.bus),
    m_rightMotor(rightIndexer.id, rightIndexer.bus),
    m_centerMotor(centerIndexer.id, centerIndexer.bus),
    m_leftVoltage(0_V),
    m_rightVoltage(0_V),
    m_centerVoltage(0_V),
    m_leftRequest(ctre::phoenix6::controls::VelocityTorqueCurrentFOC{0_rpm}.WithSlot(0)),
    m_rightRequest(leftIndexer.id, ctre::phoenix6::signals::MotorAlignmentValue::Aligned),
    m_centerRequest(ctre::phoenix6::controls::VelocityTorqueCurrentFOC{0_rpm}.WithSlot(0))
    {
        m_leftConfigs.MotorOutput.Inverted = Constants::Indexer::kLeftMotorInverted ? 
        ctre::phoenix6::signals::InvertedValue::Clockwise_Positive : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
        
        m_rightConfigs.MotorOutput.Inverted = Constants::Indexer::kRightMotorInverted ? 
        ctre::phoenix6::signals::InvertedValue::Clockwise_Positive : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

        m_rightConfigs.MotorOutput.Inverted = Constants::Indexer::kCenterMotorInverted ? 
        ctre::phoenix6::signals::InvertedValue::Clockwise_Positive : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

        // m_rightConfigs.Slot0.kP = Constants::Indexer::kP;
        // m_rightConfigs.Slot0.kP = Constants::Indexer::kI;
        // m_rightConfigs.Slot0.kP = Constants::Indexer::kD;

        m_leftConfigs.Slot0.kP = Constants::Indexer::kP;
        m_leftConfigs.Slot0.kI = Constants::Indexer::kI;
        m_leftConfigs.Slot0.kD = Constants::Indexer::kD;

        // m_centerConfigs.Slot0.kP = Constants::Indexer::centerkP;
        // m_centerConfigs.Slot0.kP = Constants::Indexer::centerkI;
        // m_centerConfigs.Slot0.kP = Constants::Indexer::centerkD;



        // m_leftConfigs.CurrentLimits.SupplyCurrentLimit = 20_A;
        // m_centerConfigs.CurrentLimits.SupplyCurrentLimit = 20_A;
        // m_rightConfigs.CurrentLimits.SupplyCurrentLimit = 20_A;

        m_rightConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;        
        m_leftConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
        m_centerConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;

        m_leftMotor.GetConfigurator().Apply(m_leftConfigs);
        m_rightMotor.GetConfigurator().Apply(m_rightConfigs);
        m_centerMotor.GetConfigurator().Apply(m_centerConfigs);
    }

void CTREIndexerIO::UpdateInputs(IndexerIOInputs &inputs){

}

void CTREIndexerIO::SetVoltage(units::volt_t voltage, units::angular_velocity::turns_per_second_t rpm) {
    // m_rightMotor.SetControl(m_rightVoltage.WithOutput(voltage));
    // m_leftMotor.SetControl(m_leftVoltage.WithOutput(voltage));

    m_rightMotor.SetControl(m_rightRequest);
    m_leftMotor.SetControl(m_leftRequest.WithVelocity(rpm));

    m_centerMotor.SetControl(m_centerVoltage.WithOutput(voltage));
}