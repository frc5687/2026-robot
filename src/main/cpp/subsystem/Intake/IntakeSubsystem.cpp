#include "subsystem/intake/IntakeSubsystem.h"

#include "subsystem/LoggedSubsystem.h"
#include "subsystem/intake/IntakeRollerIO.h"
#include "subsystem/intake/linearintake/LinearIntakeIO.h"
#include "units/length.h"
#include <memory>

IntakeSubsystem::IntakeSubsystem(std::unique_ptr<IntakeRollerIO> io) 
    : LoggedSubsystem("Intake Roller"), m_io(std::move(io)){}

void IntakeSubsystem::UpdateInputs() {m_io->UpdateInputs(m_inputs);}

void IntakeSubsystem::SetVoltage(units::volt_t voltage) {
    m_io ->SetVoltage(voltage);
}

void IntakeSubsystem::LogTelemetry(){

}
    