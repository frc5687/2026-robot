// Team 5687 2026

#include "subsystem/lights/LightsSubsystem.h"

#include <units/math.h>

#include "subsystem/lights/LightsIO.h"


LightsSubsystem::LightsSubsystem(std::unique_ptr<LightsIO> io)
    : LoggedSubsystem("Lights"), m_io(std::move(io)) {}

void LightsSubsystem::UpdateInputs() {
  m_io->UpdateInputs(m_inputs);
}

void LightsSubsystem::LogTelemetry() {
 
}
