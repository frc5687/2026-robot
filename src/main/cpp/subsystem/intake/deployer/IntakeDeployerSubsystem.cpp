// Team 5687 2026

#include "subsystem/intake/deployer/IntakeDeployerSubsystem.h"

#include <units/math.h>

#include "Constants.h"

using namespace Constants::IntakeDeployer;

IntakeDeployerSubsystem::IntakeDeployerSubsystem(
    std::unique_ptr<IntakeDeployerIO> io)
    : LoggedSubsystem("IntakeDeployer"), m_io(std::move(io)) {}

static units::turn_t ExtensionToMotorTurns(units::meter_t extension) {
  return units::turn_t{extension.value() /
                       Constants::IntakeDeployer::kRotationToMeter.value() *
                       Constants::IntakeDeployer::kGearRatio};
}

void IntakeDeployerSubsystem::Deploy() {
  m_io->SetPosition(ExtensionToMotorTurns(kDeployedExtension));
}

void IntakeDeployerSubsystem::Retract() {
  m_io->SetPosition(ExtensionToMotorTurns(kRetractedExtension));
}

void IntakeDeployerSubsystem::SetPosition(units::meter_t extension) {
  m_io->SetPosition(ExtensionToMotorTurns(extension));
}

units::meter_t IntakeDeployerSubsystem::GetPosition() const {
  return units::meter_t{(m_inputs.motorPosition / kGearRatio).value() *
                        kRotationToMeter.value()};
}

void IntakeDeployerSubsystem::SetVoltage(units::volt_t voltage) {
  m_io->SetVoltage(voltage);
}

void IntakeDeployerSubsystem::ZeroPosition() { m_io->ZeroPosition(); }

void IntakeDeployerSubsystem::Stop() { m_io->Stop(); }

bool IntakeDeployerSubsystem::IsDeployed() const {
  return units::math::abs(GetPosition() - kDeployedExtension) <
         kExtensionTolerance;
}

bool IntakeDeployerSubsystem::IsRetracted() const {
  return units::math::abs(GetPosition() - kRetractedExtension) <
         kExtensionTolerance;
}

void IntakeDeployerSubsystem::UpdateInputs() { m_io->UpdateInputs(m_inputs); }

void IntakeDeployerSubsystem::LogTelemetry() {
  Log("MotorPosition", m_inputs.motorPosition.value());
  Log("MotorVelocity", m_inputs.motorVelocity.value());
  Log("AppliedVolts", m_inputs.appliedVolts.value());
  Log("StatorCurrent", m_inputs.statorCurrent.value());
  Log("SupplyCurrent", m_inputs.supplyCurrent.value());
  Log("ForwardLimitHit", m_inputs.forwardLimitHit);
  Log("ReverseLimitHit", m_inputs.reverseLimitHit);
  Log("ExtensionMeters", GetPosition().value());
  Log("IsDeployed", IsDeployed());
  Log("IsRetracted", IsRetracted());
}
