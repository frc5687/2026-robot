// Team 5687 2026

#include "utils/MatchTracker.h"

#include <frc/DriverStation.h>

#include <string>

#include "frc/Timer.h"
#include "utils/Logger.h"

void MatchTracker::OnDisableInit() {
  m_matchTimer.Stop();
  m_matchTimer.Reset();
  m_matchOffset = 0_s;
  m_forcedMode = MatchPhase::Disabled;
  m_autoWinner.reset();
  m_currentAlliance.reset();
}

void MatchTracker::OnAutonomousInit() {
  m_matchTimer.Reset();
  m_matchTimer.Start();
  m_matchOffset = 0_s;
  m_forcedMode = MatchPhase::Autonomous;
}

void MatchTracker::OnTeleopInit() {
  m_matchTimer.Reset();
  m_matchTimer.Start();
  m_matchOffset = kAutoDuration;
  m_forcedMode = MatchPhase::Transition;
}

void MatchTracker::OnTestInit() {
  m_matchTimer.Stop();
  m_matchTimer.Reset();
  m_matchOffset = 0_s;
  m_forcedMode = MatchPhase::Test;
  m_currentAlliance.reset();
  m_autoWinner.reset();
}

void MatchTracker::UpdateAllianceState() {
  if (!m_autoWinner.has_value()) {
    auto gameDataWinner = ParseAllianceFromGameData();
    if (gameDataWinner.has_value()) {
      m_autoWinner = gameDataWinner;
    }
  }

  m_currentAlliance = frc::DriverStation::GetAlliance();
}

units::second_t MatchTracker::GetMatchElapsed() const {
  if (m_forcedMode == MatchPhase::Disabled || m_forcedMode == MatchPhase::Test) {
    return m_matchOffset;
  }

  auto elapsed = m_matchOffset + m_matchTimer.Get();
  return units::math::min(elapsed, kTotalMatchDuration);
}

units::second_t MatchTracker::GetMatchRemaining() const {
  return units::math::max(0_s, kTotalMatchDuration - GetMatchElapsed());
}

units::second_t MatchTracker::GetPhaseRemaining() const {
  const auto elapsed = GetMatchElapsed();

  switch (GetMatchPhase()) {
    case MatchPhase::Autonomous:
      return units::math::max(0_s, kAutoDuration - elapsed);

    case MatchPhase::Transition:
      return units::math::max(0_s, (kAutoDuration + kTransitionDuration) - elapsed);

    case MatchPhase::Shift1:
      return units::math::max(0_s, (kAutoDuration + kTransitionDuration+ kShiftDuration) - elapsed);

    case MatchPhase::Shift2:
      return units::math::max(0_s, (kAutoDuration + kTransitionDuration+ 2 * kShiftDuration) - elapsed);

    case MatchPhase::Shift3:
      return units::math::max(0_s, (kAutoDuration+ kTransitionDuration + 3 * kShiftDuration) - elapsed);

    case MatchPhase::Shift4:
      return units::math::max(0_s, (kAutoDuration + kTransitionDuration+ 4 * kShiftDuration) - elapsed);

    case MatchPhase::Endgame:
      return units::math::max(0_s, kTotalMatchDuration - elapsed);

    case MatchPhase::Disabled:
    case MatchPhase::Test:
      return 0_s;
  }

  return 0_s;
}

MatchTracker::MatchPhase MatchTracker::GetMatchPhase() const {
  return GetMatchPhaseAt(GetMatchElapsed());
}

void MatchTracker::SetCurrentAlliance(frc::DriverStation::Alliance alliance) {
  m_currentAlliance = alliance;
}

std::optional<frc::DriverStation::Alliance>
MatchTracker::GetCurrentAlliance() const {
  return m_currentAlliance;
}

void MatchTracker::SetAutoWinnerFromCurrentAlliance(
    bool currentAllianceWon) {
  auto alliance =
      m_currentAlliance.has_value()
          ? m_currentAlliance
          : frc::DriverStation::GetAlliance();

  if (!alliance.has_value()) {
    return;
  }

  if (currentAllianceWon) {
    m_autoWinner = *alliance;
    return;
  }

  m_autoWinner = (*alliance == frc::DriverStation::Alliance::kRed
                     ? frc::DriverStation::Alliance::kBlue
                     : frc::DriverStation::Alliance::kRed);
}

std::optional<frc::DriverStation::Alliance> MatchTracker::GetAutoWinner() const {
  return m_autoWinner;
}

std::optional<frc::DriverStation::Alliance> MatchTracker::GetActiveAlliance() const {
  return GetActiveAllianceForPhase(GetMatchPhase());
}

std::string MatchTracker::GetCurrentShootingAlliance() const {
  switch (GetMatchPhase()) {
  case MatchPhase::Autonomous:
  case MatchPhase::Endgame:
    return "Both";
  case MatchPhase::Shift1:
  case MatchPhase::Shift2:
  case MatchPhase::Shift3:
  case MatchPhase::Shift4:
    if (auto activeAlliance = GetActiveAlliance(); activeAlliance.has_value()) {
      return MatchAllianceToString(activeAlliance.value());
    }
    return "None";
  case MatchPhase::Transition:
  case MatchPhase::Disabled:
  case MatchPhase::Test:
    return "None";
  }
  return "None";
}

bool MatchTracker::CanAllianceScore(frc::DriverStation::Alliance alliance) const {
  auto activeAlliance = GetActiveAlliance();

  switch (GetMatchPhase()) {
  case MatchPhase::Autonomous:
    return true;
  case MatchPhase::Shift1:
  case MatchPhase::Shift2:
  case MatchPhase::Shift3:
  case MatchPhase::Shift4:
    return activeAlliance.has_value() && activeAlliance.value() == alliance;
  case MatchPhase::Endgame:
    return true;
  case MatchPhase::Transition:
  case MatchPhase::Disabled:
  case MatchPhase::Test:
    return false;
  }
  return false;
}

bool MatchTracker::ShouldPrespinForAlliance(
    frc::DriverStation::Alliance alliance, units::second_t leadTime) const {
  if (CanAllianceScore(alliance)) {
    return true;
  }

  auto futureElapsed =
      units::math::min(GetMatchElapsed() + leadTime, kTotalMatchDuration);
  MatchPhase futurePhase = GetMatchPhaseAt(futureElapsed);
  if (futurePhase == MatchPhase::Endgame || futurePhase == MatchPhase::Disabled ||
      futurePhase == MatchPhase::Test) {
    return false;
  }

  auto futureAlliance = GetActiveAllianceForPhase(futurePhase);
  return futureAlliance.has_value() && futureAlliance.value() == alliance;
}

MatchTracker::MatchPhase
MatchTracker::GetMatchPhaseAt(units::second_t elapsed) const {
  if (m_forcedMode == MatchPhase::Disabled || m_forcedMode == MatchPhase::Test) {
    return m_forcedMode;
  }

  if (elapsed < kAutoDuration) {
    return MatchPhase::Autonomous;
  }

  auto teleopElapsed = elapsed - kAutoDuration;
  if (teleopElapsed < kTransitionDuration) {
    return MatchPhase::Transition;
  }
  teleopElapsed -= kTransitionDuration;

  if (teleopElapsed < kShiftDuration) {
    return MatchPhase::Shift1;
  }
  teleopElapsed -= kShiftDuration;

  if (teleopElapsed < kShiftDuration) {
    return MatchPhase::Shift2;
  }
  teleopElapsed -= kShiftDuration;

  if (teleopElapsed < kShiftDuration) {
    return MatchPhase::Shift3;
  }
  teleopElapsed -= kShiftDuration;

  if (teleopElapsed < kShiftDuration) {
    return MatchPhase::Shift4;
  }

  return MatchPhase::Endgame;
}

std::optional<frc::DriverStation::Alliance>
MatchTracker::GetActiveAllianceForPhase(MatchPhase phase) const {
  switch (phase) {
  case MatchPhase::Autonomous:
  case MatchPhase::Transition:
  case MatchPhase::Endgame:
  case MatchPhase::Disabled:
  case MatchPhase::Test:
    return std::nullopt;
  case MatchPhase::Shift1:
  case MatchPhase::Shift3: {
    if (!m_autoWinner.has_value()) {
      return std::nullopt;
    }

    return m_autoWinner.value() == frc::DriverStation::Alliance::kRed
               ? frc::DriverStation::Alliance::kBlue
               : frc::DriverStation::Alliance::kRed;
  }
  case MatchPhase::Shift2:
  case MatchPhase::Shift4:
    return m_autoWinner;
  }

  return std::nullopt;
}

std::string MatchTracker::MatchPhaseToString(MatchPhase phase) const {
  switch (phase) {
  case MatchPhase::Disabled:
    return "Disabled";
  case MatchPhase::Autonomous:
    return "Autonomous";
  case MatchPhase::Transition:
    return "Transition";
  case MatchPhase::Shift1:
    return "Shift1";
  case MatchPhase::Shift2:
    return "Shift2";
  case MatchPhase::Shift3:
    return "Shift3";
  case MatchPhase::Shift4:
    return "Shift4";
  case MatchPhase::Endgame:
    return "Endgame";
  case MatchPhase::Test:
    return "Test";
  }
  return "Unknown";
}

std::string MatchTracker::MatchAllianceToString(
    frc::DriverStation::Alliance alliance) const {
  switch (alliance) {
  case frc::DriverStation::Alliance::kRed:
    return "Red";
  case frc::DriverStation::Alliance::kBlue:
    return "Blue";
  default:
    return "Unknown";
  }
}

std::optional<frc::DriverStation::Alliance>
MatchTracker::ParseAllianceFromGameData() const {
  std::string gameData = frc::DriverStation::GetGameSpecificMessage();
  if (gameData.length() > 0) {
    switch (gameData[0]) {
    case 'R':
      return frc::DriverStation::Alliance::kRed;
    case 'B':
      return frc::DriverStation::Alliance::kBlue;
    default:
      return std::nullopt;
    }
  }
  return std::nullopt;
}

void MatchTracker::LogState(units::second_t timestamp) {
  auto currentAlliance = GetCurrentAlliance();
  auto autoWinner = GetAutoWinner();
  auto activeAlliance = GetActiveAlliance();

  Logger::Instance().Log("MatchTracker/Timestamp", timestamp.value());
  Logger::Instance().Log("MatchTracker/MatchElapsedSec",
                         GetMatchElapsed().value());
  Logger::Instance().Log("MatchTracker/MatchRemainingSec",
                         GetMatchRemaining().value());
  Logger::Instance().Log("MatchTracker/PhaseRemainingSec",
                         GetPhaseRemaining().value());
  Logger::Instance().Log("MatchTracker/MatchPhase",
                         MatchPhaseToString(GetMatchPhase()));
  Logger::Instance().Log(
      "MatchTracker/CurrentAlliance",
      currentAlliance.has_value() ? MatchAllianceToString(*currentAlliance) : "None");
  Logger::Instance().Log(
      "MatchTracker/AutoWinner",
      autoWinner.has_value() ? MatchAllianceToString(*autoWinner) : "None");
  Logger::Instance().Log(
      "MatchTracker/ActiveAlliance",
      activeAlliance.has_value() ? MatchAllianceToString(*activeAlliance) : "None");
  Logger::Instance().Log("MatchTracker/CurrentShootingAlliance",
                         GetCurrentShootingAlliance());
  Logger::Instance().Log("MatchTracker/ScoringEnabled/Red",
                         CanAllianceScore(frc::DriverStation::Alliance::kRed));
  Logger::Instance().Log("MatchTracker/ScoringEnabled/Blue",
                         CanAllianceScore(frc::DriverStation::Alliance::kBlue));
  Logger::Instance().Log("MatchTracker/PrespinEnabled/Red",
                         ShouldPrespinForAlliance(
                             frc::DriverStation::Alliance::kRed));
  Logger::Instance().Log("MatchTracker/PrespinEnabled/Blue",
                         ShouldPrespinForAlliance(
                             frc::DriverStation::Alliance::kBlue));
}
