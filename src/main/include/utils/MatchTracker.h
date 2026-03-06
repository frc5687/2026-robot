#pragma once

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <units/time.h>

#include <optional>
#include <string>

class MatchTracker {
public:
  enum class MatchPhase {
    Disabled,
    Autonomous,
    Transition,
    Shift1,
    Shift2,
    Shift3,
    Shift4,
    Endgame,
    Test,
  };

  static constexpr units::second_t kAutoDuration = 20_s;
  static constexpr units::second_t kTransitionDuration = 10_s;
  static constexpr units::second_t kShiftDuration = 25_s;
  static constexpr units::second_t kTotalMatchDuration = 160_s;
  static constexpr units::second_t kFlywheelPrespinLeadTime = 3_s;

  static MatchTracker &Instance() {
    static MatchTracker inst;
    return inst;
  }

  void OnDisableInit();
  void OnAutonomousInit();
  void OnTeleopInit();
  void OnTestInit();

  units::second_t GetMatchElapsed() const;
  units::second_t GetMatchRemaining() const;
  MatchPhase GetMatchPhase() const;

  void UpdateAllianceState();
  void SetCurrentAlliance(frc::DriverStation::Alliance alliance);
  void SetAutoWinnerFromCurrentAlliance(bool currentAllianceWon);
  std::optional<frc::DriverStation::Alliance> GetAutoWinner() const;
  std::optional<frc::DriverStation::Alliance> GetActiveAlliance() const;
  std::string GetCurrentShootingAlliance() const;
  bool CanAllianceScore(frc::DriverStation::Alliance alliance) const;
  bool ShouldPrespinForAlliance(
      frc::DriverStation::Alliance alliance,
      units::second_t leadTime = kFlywheelPrespinLeadTime) const;

  void LogState(units::second_t timestamp);

  std::string MatchPhaseToString(MatchPhase phase) const;
  std::string MatchAllianceToString(
      frc::DriverStation::Alliance alliance) const;

private:
  MatchTracker() = default;

  MatchPhase GetMatchPhaseAt(units::second_t elapsed) const;
  std::optional<frc::DriverStation::Alliance> GetActiveAllianceForPhase(
      MatchPhase phase) const;
  std::optional<frc::DriverStation::Alliance> GetCurrentAlliance() const;
  std::optional<frc::DriverStation::Alliance> ParseAllianceFromGameData() const;

  frc::Timer m_matchTimer;
  MatchPhase m_forcedMode{MatchPhase::Disabled};
  units::second_t m_matchOffset{0_s};
  std::optional<frc::DriverStation::Alliance> m_currentAlliance;
  std::optional<frc::DriverStation::Alliance> m_autoWinner;
};
