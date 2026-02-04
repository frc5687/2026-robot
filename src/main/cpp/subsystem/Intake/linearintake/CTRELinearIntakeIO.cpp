#include "subsystem/intake/linearintake/CTRELinearIntakeIO.h"

#include "ctre/phoenix6/StatusSignal.hpp"
#include "subsystem/intake/linearintake/LinearIntakeIO.h"
#include "subsystem/intake/linearintake/LinearIntake.h"

#include "units/angle.h"
#include "utils/CANDevice.h"

using namespace Constants::LinearIntake;
CTRELinearIntakeIO::CTRELinearIntakeIO(const CANDevice &linearMotor):
    m_linearMotor(linearMotor.id, linearMotor.bus),
    m_linearController(0_tr),
    m_linearMotorSupplyAmps(m_linearMotor.GetSupplyCurrent()),
    m_linearMotorPosition(m_linearMotor.GetPosition()),
    m_linearMotorVelocity(m_linearMotor.GetVelocity()),
    m_batchStatusSignals{&m_linearMotorSupplyAmps, &m_linearMotorPosition, &m_linearMotorVelocity}
    {
        m_linearConfigs.MotorOutput.Inverted = Constants::LinearIntake::kLinearMotorInverted ?
        ctre::phoenix6::signals::InvertedValue::Clockwise_Positive : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;


        m_linearConfigs.Voltage.PeakForwardVoltage = 12_V;
        m_linearConfigs.Voltage.PeakReverseVoltage = -12_V;

        m_linearConfigs.Slot0.kP = Constants::LinearIntake::kP;
        m_linearConfigs.Slot0.kI = Constants::LinearIntake::kI;
        m_linearConfigs.Slot0.kD = Constants::LinearIntake::kD;

        m_linearConfigs.MotionMagic.MotionMagicCruiseVelocity = 
             kMaxVelocity / kDrumRadius *
            kGearRatio * 1_tr;
        m_linearConfigs.MotionMagic.MotionMagicAcceleration = kMaxAccel / kDrumRadius *
            kGearRatio * 1_tr;
        m_linearConfigs.MotionMagic.MotionMagicJerk = 10000_tr_per_s_cu;

        m_linearMotor.GetConfigurator().Apply(m_linearConfigs);
};

void CTRELinearIntakeIO::UpdateInputs(LinearIntakeIOInputs &inputs){
    ctre::phoenix6::BaseStatusSignal::RefreshAll(m_batchStatusSignals);

    inputs.motorPosition = m_linearMotorPosition.GetValue();
    inputs.motorVelocity = m_linearMotorVelocity.GetValue();


    inputs.linearIntakePosition = inputs.motorPosition *
                                      kCircumference /
                                      kGearRatio / 1_tr;

    inputs.linearIntakeVelocity = inputs.motorVelocity *
                                      kCircumference /
                                      kGearRatio / 1_tr;

}

void CTRELinearIntakeIO::SetPosition(units::meter_t desiredMeters){

     units::turn_t motorTurns = desiredMeters /
                                 kCircumference *
                                 kGearRatio * 1_tr;
    m_linearMotor.SetControl(m_linearController.WithPosition(motorTurns).WithSlot(0));
}

