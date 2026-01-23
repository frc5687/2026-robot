#include "subsystem/intake/linearintake/LinearIntake.h"
#include "subsystem/LoggedSubsystem.h"
#include "subsystem/intake/linearintake/LinearIntakeIO.h"


LinearIntake::LinearIntake(std::unique_ptr<LinearIntakeIO>io):
LoggedSubsystem("LinearIntake"), m_io(std::move(io)) {}

void LinearIntake::UpdateInputs() {m_io->UpdateInputs(m_inputs);}

void LinearIntake::SetPosition(units::meter_t meters){
    m_io ->SetPosition(meters);
}

void LinearIntake::LogTelemetry(){

}