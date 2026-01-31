#pragma once

#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/controls/PositionVoltage.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "subsystem/turret/TurretIO.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "utils/CANDevice.h"
#include "TurretIO.h"
#include "ctre/phoenix6/TalonFX.hpp"

using namespace ctre::phoenix6;

class CTRETurretIO : public TurretIO {
  public:
    CTRETurretIO(const CANDevice &motor);
    void UpdateInputs(TurretIOInputs& inputs) override;
    void SetTurretAngle(units::radian_t angle) override;

  private:
    hardware::TalonFX m_motor;

    configs::TalonFXConfiguration m_config{};

    controls::PositionVoltage m_controller;

    StatusSignal<units::turns_per_second_t> m_motorVelocitySignal;
    StatusSignal<units::turn_t> m_motorPositionSignal;

    std::array<BaseStatusSignal*, 2> m_batchSignals;
};
