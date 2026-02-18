#include "subsystem/indexer/CTREIndexerIO.h"

#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "subsystem/indexer/IndexerConstants.h"
#include "subsystem/indexer/IndexerIO.h"
#include "units/angular_velocity.h"
#include "units/voltage.h"
#include "utils/CANDevice.h"


CTREIndexerIO::CTREIndexerIO(const CANDevice &rightIndexer, const CANDevice &leftIndexer, const CANDevice &leftfeeder,
const CANDevice &rightfeeder):
    m_leftkickerMotor(leftIndexer.id, leftIndexer.bus),
    m_rightkickerMotor(rightIndexer.id, rightIndexer.bus),
    m_leftfeederMotor(leftfeeder.id, rightfeeder.bus),
    m_rightfeederMotor(rightfeeder.id, rightfeeder.bus),

    m_leftkickerRequest(ctre::phoenix6::controls::VelocityTorqueCurrentFOC{0_rpm}.WithSlot(0)),
    m_rightkickerRequest(leftIndexer.id, ctre::phoenix6::signals::MotorAlignmentValue::Opposed),
    m_leftfeederRequest(ctre::phoenix6::controls::VelocityTorqueCurrentFOC{0_rpm}.WithSlot(0)),
    m_rightfeederRequest(leftIndexer.id, ctre::phoenix6::signals::MotorAlignmentValue::Opposed)
    

    {
        m_leftConfigs.MotorOutput.Inverted = Constants::Indexer::kLeftMotorInverted ? 
        ctre::phoenix6::signals::InvertedValue::Clockwise_Positive : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
        
        m_rightConfigs.MotorOutput.Inverted = Constants::Indexer::kRightMotorInverted ? 
        ctre::phoenix6::signals::InvertedValue::Clockwise_Positive : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

        m_centerConfigs.MotorOutput.Inverted = Constants::Indexer::kCenterMotorInverted ? 
        ctre::phoenix6::signals::InvertedValue::Clockwise_Positive : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

        // m_rightConfigs.Slot0.kP = Constants::Indexer::kP;
        // m_rightConfigs.Slot0.kP = Constants::Indexer::kI;
        // m_rightConfigs.Slot0.kP = Constants::Indexer::kD;

        m_leftConfigs.Slot0.kP = Constants::Indexer::kP;
        m_leftConfigs.Slot0.kI = Constants::Indexer::kI;
        m_leftConfigs.Slot0.kD = Constants::Indexer::kD;

        m_centerConfigs.Slot0.kP = Constants::Indexer::centerkP;
        m_centerConfigs.Slot0.kP = Constants::Indexer::centerkI;
        m_centerConfigs.Slot0.kP = Constants::Indexer::centerkD;



        // m_leftConfigs.CurrentLimits.SupplyCurrentLimit = 20_A;
        // m_centerConfigs.CurrentLimits.SupplyCurrentLimit = 20_A;
        // m_rightConfigs.CurrentLimits.SupplyCurrentLimit = 20_A;

        // m_rightConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;        
        
        m_centerConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;

        m_leftfeederMotor.GetConfigurator().Apply(m_leftConfigs);
        m_leftkickerMotor.GetConfigurator().Apply(m_centerConfigs);
    }

void CTREIndexerIO::UpdateInputs(IndexerIOInputs &inputs){

}

void CTREIndexerIO::SetFeederRPM(units::angular_velocity::turns_per_second_t rpm) {
    // m_rightMotor.SetControl(m_rightVoltage.WithOutput(voltage));
    // m_leftMotor.SetControl(m_leftVoltage.WithOutput(voltage));

    m_leftfeederMotor.SetControl(m_leftfeederRequest.WithVelocity(rpm));
    m_rightfeederMotor.SetControl(m_rightfeederRequest);


}

void CTREIndexerIO::SetKickerRPM(units::angular_velocity::turns_per_second_t rpm) {
    // m_rightMotor.SetControl(m_rightVoltage.WithOutput(voltage));
    // m_leftMotor.SetControl(m_leftVoltage.WithOutput(voltage));

    m_leftkickerMotor.SetControl(m_leftkickerRequest.WithVelocity(rpm));
    m_rightkickerMotor.SetControl(m_rightkickerRequest);


}