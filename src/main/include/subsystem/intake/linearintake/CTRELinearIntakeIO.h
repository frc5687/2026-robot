#pragma once

#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/MotionMagicTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "subsystem/intake/CTREIntakeRollerIO.h"
#include "subsystem/intake/linearintake/LinearIntakeIO.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"

class CTRELinearIntakeIO : public LinearIntakeIO {
    public:
    CTRELinearIntakeIO(const CANDevice &linearMotor);
    void UpdateInputs(LinearIntakeIOInputs& inputs) override;
    void SetPosition(units::meter_t meters) override;

    private:
        ctre::phoenix6::hardware::TalonFX m_linearMotor;

        ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC m_linearController;

        ctre::phoenix6::configs::TalonFXConfiguration m_linearConfigs{};

        ctre::phoenix6::StatusSignal<units::ampere_t> m_linearMotorSupplyAmps;
        ctre::phoenix6::StatusSignal<units::turn_t> m_linearMotorPosition;
        ctre::phoenix6::StatusSignal<units::turns_per_second_t> m_linearMotorVelocity;

        std::array<ctre::phoenix6::BaseStatusSignal*, 3> m_batchStatusSignals;
};