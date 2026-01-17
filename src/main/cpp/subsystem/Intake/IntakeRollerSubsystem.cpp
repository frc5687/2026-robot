#include "subsystem/intake/IntakeRoller.h"

#include "subsystem/LoggedSubsystem.h"
#include "subsystem/intake/IntakeRollerIO.h"

IntakeRoller::IntakeRoller(std::unique_ptr<IntakeRollerIO> io) 
    : LoggedSubsystem("IntakeRoller"), m_io(std::move(io)) {}

void IntakeRoller::UpdateInputs() {m_io->UpdateInputs(m_inputs);}

void IntakeRoller::SetVoltage(units::volt_t voltage) {
    m_io ->SetVoltage(voltage);
}

void IntakeRoller::LogTelemetry(){

}
    