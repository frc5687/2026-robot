#pragma once

#include "IntakeRollerIO.h"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/VoltageOut.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "utils/CANDevice.h"


class CTREIntakeRollerIO : public IntakeRollerIO {
    public:
    CTREIntakeRollerIO(const CANDevice &rightMotor, const CANDevice &leftMotor);
    void UpdateInputs(IntakeRollerIOInputs& inputs) override;
    void SetIntakeRPM(units::radians_per_second_t desiredRPM) override;
    void SetVoltage(units::volt_t voltage) override;

    private:
        ctre::phoenix6::hardware::TalonFX m_leftMotor;
        ctre::phoenix6::hardware::TalonFX m_rightMotor;

        ctre::phoenix6::controls::VoltageOut m_leftVoltage;
        ctre::phoenix6::controls::VoltageOut m_rightVoltage;

        ctre::phoenix6::configs::TalonFXConfiguration m_leftConfigs{};
        ctre::phoenix6::configs::TalonFXConfiguration m_rightConfigs{};
};