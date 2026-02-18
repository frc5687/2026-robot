#pragma once

#include "IndexerIO.h"
#include "IndexerConstants.h"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/Follower.hpp"
#include "ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/VoltageOut.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "units/angular_velocity.h"
#include "utils/CANDevice.h"


class CTREIndexerIO : public IndexerIO {
    public:
    CTREIndexerIO(const CANDevice &rightMotor, const CANDevice &leftMotor, const CANDevice &leftfeeder, const CANDevice &rightfeeder);
    void UpdateInputs(IndexerIOInputs& inputs) override;
    void SetFeederRPM(units::angular_velocity::turns_per_second_t rpm) override;
    void SetKickerRPM(units::angular_velocity::turns_per_second_t rpm) override;

    private:
        ctre::phoenix6::hardware::TalonFX m_leftkickerMotor;
        ctre::phoenix6::hardware::TalonFX m_rightkickerMotor;
        ctre::phoenix6::hardware::TalonFX m_leftfeederMotor;
        ctre::phoenix6::hardware::TalonFX m_rightfeederMotor;


        // ctre::phoenix6::controls::VoltageOut m_leftVoltage;
        // ctre::phoenix6::controls::VoltageOut m_rightVoltage;
        // ctre::phoenix6::controls::VoltageOut m_centerVoltage;

        ctre::phoenix6::controls::VelocityTorqueCurrentFOC m_leftkickerRequest;
        ctre::phoenix6::controls::Follower m_rightkickerRequest;
        ctre::phoenix6::controls::VelocityTorqueCurrentFOC m_leftfeederRequest;
        ctre::phoenix6::controls::Follower m_rightfeederRequest;



        ctre::phoenix6::configs::TalonFXConfiguration m_leftConfigs{};
        ctre::phoenix6::configs::TalonFXConfiguration m_rightConfigs{};
        ctre::phoenix6::configs::TalonFXConfiguration m_centerConfigs{};

};