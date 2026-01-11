#include "frc/simulation/LinearSystemSim.h"
#include "subsystem/turret/SimTurretIO.h"
#include "subsystem/turret/TurretConstants.h"
#include "frc/system/plant/LinearSystemId.h"
#include "frc/Timer.h"

using namespace Constants::Turret;

SimTurretIO::SimTurretIO() :
    m_turretSim(frc::LinearSystemId::DCMotorSystem(
        kMotor,
        kInertia,
        kGearRatio
    ), kMotor, {0.001, 0.001}),
    m_controller(kP, kI, kD)
{
}

void SimTurretIO::UpdateInputs(TurretIOInputs& inputs) {
    constexpr auto kDt = 20_ms;

    m_turretSim.Update(kDt);

    auto turretPosition = m_turretSim.GetAngularPosition();
    auto turretVelocity = m_turretSim.GetAngularVelocity();

    // Motor position/velocity is turret * gear ratio
    inputs.motorPosition = (turretPosition * kGearRatio) / (2.0 * std::numbers::pi * 1_rad) * 1_tr;
    inputs.motorVelocity = (turretVelocity * kGearRatio) / (2.0 * std::numbers::pi * 1_rad_per_s) * 1_tps;

    // Turret angle/velocity
    inputs.angle = turretPosition;
    inputs.angularVelocity = turretVelocity;
    inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void SimTurretIO::SetTurretAngle(units::radian_t angle) {
}
