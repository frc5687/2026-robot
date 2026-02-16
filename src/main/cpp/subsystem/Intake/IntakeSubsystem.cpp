#include "subsystem/intake/IntakeSubsystem.h"

#include "subsystem/LoggedSubsystem.h"
#include "subsystem/intake/IntakeRollerIO.h"
#include "subsystem/intake/linearintake/LinearIntakeIO.h"
#include "units/length.h"
#include <memory>

IntakeSubsystem::IntakeSubsystem(std::unique_ptr<LinearIntakeIO> lio, std::unique_ptr<IntakeRollerIO> rio) 
    : LoggedSubsystem("Intake"), m_Lio(std::move(lio)), m_Rio(std::move(rio)){}

void IntakeSubsystem::UpdateInputs() {m_Rio->UpdateInputs(m_Rinputs); m_Lio->UpdateInputs(m_Linputs);}

void IntakeSubsystem::SetVoltage(units::volt_t voltage) {
    m_Rio ->SetVoltage(voltage);
}

void IntakeSubsystem::SetPosition(units::meter_t meter){
    m_Lio ->SetPosition(meter);
}

void IntakeSubsystem::LogTelemetry(){

}
    