// Team 5687 2026

#include "subsystem/intake/deployer/IntakeDeployerSubsystem.h"

#include <algorithm>
#include <units/math.h>

#include "subsystem/intake/IntakeConstants.h"

using namespace Constants::IntakeDeployer;

IntakeDeployerSubsystem::IntakeDeployerSubsystem(
    std::unique_ptr<IntakeDeployerIO> io)
    : LoggedSubsystem("IntakeDeployer"), m_io(std::move(io)) {}

static units::turn_t ExtensionToMotorTurns(units::meter_t extension) {
  return units::turn_t{extension.value() / kMetersPerMotorRotation};
}

void IntakeDeployerSubsystem::FullyExtend() {
  CancelSlowRetract();
  DisableCompliantHold();
  m_io->SetPosition(ExtensionToMotorTurns(kFullyExtend));
}

void IntakeDeployerSubsystem::Deploy() {
  CancelSlowRetract();
  m_compliantTarget = kDeployedExtension;
  m_complianceState = ComplianceState::Holding;
  m_io->SetPosition(ExtensionToMotorTurns(kDeployedExtension));
}

void IntakeDeployerSubsystem::RetractMid() {
  CancelSlowRetract();
  DisableCompliantHold();
  m_io->SetPosition(ExtensionToMotorTurns(kMidExtension));
}

void IntakeDeployerSubsystem::Retract() {
  CancelSlowRetract();
  DisableCompliantHold();
  m_io->SetPosition(ExtensionToMotorTurns(kRetractedExtension));
}

void IntakeDeployerSubsystem::SlowRetract(units::second_t duration) {
  m_slowRetract = SlowRetractState{
      .startPosition = GetPosition(),
      .duration = duration,
      .startTime = units::second_t{frc::Timer::GetFPGATimestamp()},
  };
}

void IntakeDeployerSubsystem::CancelSlowRetract() {
  m_slowRetract.reset();
}

bool IntakeDeployerSubsystem::IsSlowRetracting() const {
  return m_slowRetract.has_value();
}

void IntakeDeployerSubsystem::SetPosition(units::meter_t extension) {
  CancelSlowRetract();
  DisableCompliantHold();
  m_io->SetPosition(ExtensionToMotorTurns(extension));
}

units::meter_t IntakeDeployerSubsystem::GetPosition() const {
  return units::meter_t{m_inputs.motorPosition.value() *
                        kMetersPerMotorRotation};
}

void IntakeDeployerSubsystem::SetVoltage(units::volt_t voltage) {
  m_io->SetVoltage(voltage);
}

void IntakeDeployerSubsystem::ZeroPosition() { m_io->ZeroPosition(); }

void IntakeDeployerSubsystem::Stop() {
  CancelSlowRetract();
  DisableCompliantHold();
  m_io->Stop();
}

void IntakeDeployerSubsystem::DisableCompliantHold() {
  m_compliantTarget.reset();
  m_complianceState = ComplianceState::Holding;
}

bool IntakeDeployerSubsystem::IsDeployed() const {
  return units::math::abs(GetPosition() - kDeployedExtension) <
         kExtensionTolerance;
}

bool IntakeDeployerSubsystem::IsRetracted() const {
  return units::math::abs(GetPosition() - kRetractedExtension) <
         kExtensionTolerance;
}

void IntakeDeployerSubsystem::UpdateInputs() {
  m_io->UpdateInputs(m_inputs);

  if (m_slowRetract) {
    auto elapsed = units::second_t{frc::Timer::GetFPGATimestamp()} -
                   m_slowRetract->startTime;
    double t =
        std::clamp(static_cast<double>(elapsed / m_slowRetract->duration),
                   0.0, 1.0);
    units::meter_t target =
        m_slowRetract->startPosition * (1.0 - t) + kMidExtension * t;
    m_io->SetPosition(ExtensionToMotorTurns(target));

    if (t >= 1.0) {
      m_slowRetract.reset();
    }
  }

  if (m_compliantTarget) {
    auto targetTurns = ExtensionToMotorTurns(*m_compliantTarget);
    auto positionError = units::math::abs(targetTurns - m_inputs.motorPosition);
    auto statorCurrent = units::math::abs(m_inputs.statorCurrent);

    switch (m_complianceState) {
    case ComplianceState::Holding:
      if (statorCurrent > Compliance::kCurrentThreshold &&
          positionError > Compliance::kPositionErrorThreshold) {
        m_complianceState = ComplianceState::Yielding;
        m_io->SetVoltage(Compliance::kYieldVoltage);
      }
      break;
    case ComplianceState::Yielding:
      m_io->SetVoltage(Compliance::kYieldVoltage);
      if (statorCurrent < Compliance::kRecoverCurrentThreshold &&
          units::math::abs(m_inputs.motorVelocity) <
              Compliance::kSettledVelocity) {
        m_complianceState = ComplianceState::Holding;
        m_io->SetPosition(targetTurns);
      }
      break;
    }
  }
}

void IntakeDeployerSubsystem::LogTelemetry() {
  Log("MotorPosition", m_inputs.motorPosition.value());
  Log("MotorVelocity", m_inputs.motorVelocity.value());
  Log("AppliedVolts", m_inputs.appliedVolts.value());
  Log("Current/Leader/Supply", m_inputs.supplyCurrent.value());
  Log("ForwardLimitHit", m_inputs.forwardLimitHit);
  Log("ReverseLimitHit", m_inputs.reverseLimitHit);
  Log("ExtensionMeters", GetPosition().value());
  Log("IsDeployed", IsDeployed());
  Log("IsRetracted", IsRetracted());
  Log("ComplianceYielding",
      m_compliantTarget.has_value() &&
          m_complianceState == ComplianceState::Yielding);
  Log("StatorCurrent", m_inputs.statorCurrent.value());
}

units::ampere_t IntakeDeployerSubsystem::GetElectricalCurrentDraw() const {
  return m_inputs.supplyCurrent;
}

units::watt_t IntakeDeployerSubsystem::GetElectricalPowerDraw() const {
  return units::math::abs(m_inputs.supplyCurrent) * m_inputs.appliedVolts;
}
