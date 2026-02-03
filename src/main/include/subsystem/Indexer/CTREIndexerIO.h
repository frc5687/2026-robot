#pragma once

#include "IndexerIO.h"
#include "IndexerConstants.h"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/VoltageOut.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "utils/CANDevice.h"


class CTREIndexerIO : public IndexerIO {
    public:
    CTREIndexerIO(const CANDevice &rightMotor, const CANDevice &leftMotor, const CANDevice &centerMotor);
    void UpdateInputs(IndexerIOInputs& inputs) override;
    void SetVoltage(units::volt_t voltage) override;

    private:
        ctre::phoenix6::hardware::TalonFX m_leftMotor;
        ctre::phoenix6::hardware::TalonFX m_rightMotor;
        ctre::phoenix6::hardware::TalonFX m_centerMotor;

        ctre::phoenix6::controls::VoltageOut m_leftVoltage;
        ctre::phoenix6::controls::VoltageOut m_rightVoltage;
        ctre::phoenix6::controls::VoltageOut m_centerVoltage;


        ctre::phoenix6::configs::TalonFXConfiguration m_leftConfigs{};
        ctre::phoenix6::configs::TalonFXConfiguration m_rightConfigs{};
        ctre::phoenix6::configs::TalonFXConfiguration m_centerConfigs{};

};