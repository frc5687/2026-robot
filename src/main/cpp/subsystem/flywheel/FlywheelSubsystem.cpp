// Team 5687 2026
#include "subsystem/flywheel/FlywheelSubsystem.h"

#include <utility>

#include "Constants.h"
#include "RobotState.h"
#include "subsystem/LoggedSubsystem.h"
#include "units/angular_velocity.h"
#include "units/math.h"

FlywheelSubsystem::FlywheelSubsystem(std::unique_ptr<FlywheelIO> io)
: LoggedSubsystem("Flywheel"), m_io(std::move(io)) {}

void FlywheelSubsystem::UpdateInputs() {
    m_io->UpdateInputs(m_inputs);
    RobotState::Instance().AddFlywheelObservation(GetFlywheelState());
}

void FlywheelSubsystem::SetFlywheelRPM(
    units::revolutions_per_minute_t desiredRPM) {
    m_desiredRPM = desiredRPM;
    // desiredRPM is output-shaft; multiply by gear ratio for motor-shaft RPM.
    m_io->SetMotorRPM(desiredRPM * Constants::Flywheel::kGearRatio);
}

FlywheelState FlywheelSubsystem::GetFlywheelState() const {
    FlywheelState state;
    state.timestamp = m_inputs.timestamp;

    // motorVelocity is motor-shaft turns/s.
    // Divide by gear ratio → output-shaft turns/s.
    // Library implicitly converts turns/s → rad/s on assignment.
    state.velocity = units::radians_per_second_t{
        m_inputs.motorVelocity / Constants::Flywheel::kGearRatio};

    // motorAcceleration is motor-shaft rad/s².
    // Divide by gear ratio → output-shaft rad/s².
    state.acceleration =
        m_inputs.motorAcceleration / Constants::Flywheel::kGearRatio;

    return state;
}

bool FlywheelSubsystem::AtSetpoint() const {
    units::radians_per_second_t current = GetFlywheelState().velocity;

    // m_desiredRPM is output-shaft RPM; library converts RPM → rad/s.
    units::radians_per_second_t target{m_desiredRPM};

    return units::math::abs(current - target) <
    Constants::Flywheel::kVelocityTolerance;
}

void FlywheelSubsystem::LogTelemetry() {
    FlywheelState state = GetFlywheelState();

    Log("Motor Velocity",   m_inputs.motorVelocity.value());
    Log("Motor Accel)",   m_inputs.motorAcceleration.value());
    Log("Stator Current",     m_inputs.statorCurrent.value());
    Log("Output Velocity", state.velocity.value());
    Log("Output Accel",  state.acceleration.value());
    Log("Desired RPM",            m_desiredRPM.value());
    Log("Desired RPS", units::turns_per_second_t{m_desiredRPM}.value());
    Log("At Setpoint",            AtSetpoint());
    Log("Timestamp",          m_inputs.timestamp.value());
}
