#pragma once

#include <units/angle.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/controller/PIDController.h>

#include "subsystem/turret/TurretIO.h"

class SimTurretIO : public TurretIO {
public:
    SimTurretIO();
    ~SimTurretIO() = default;

    void UpdateInputs(TurretIOInputs& inputs) override;
    void SetTurretAngle(units::radian_t angle) override;
private:
    frc::sim::DCMotorSim m_turretSim;
    frc::PIDController m_controller;
};
