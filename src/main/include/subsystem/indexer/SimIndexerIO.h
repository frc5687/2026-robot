// Team 5687 2026

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>

#include "Constants.h"
#include "IndexerIO.h"

class SimIndexerIO : public IndexerIO {
public:
  SimIndexerIO();
  ~SimIndexerIO() override = default;

  void UpdateInputs(IndexerIOInputs &inputs) override;
  void SetVoltage(units::volt_t voltage) override;
  void SetMotorVelocity(units::turns_per_second_t rps) override;
  void Stop() override;

  /// Sim-only: inject a ball to simulate load on the indexer.
  void InjectBall();

private:
  units::volt_t CalculateClosedLoop();

  // DCMotorSim (WPILib 2025+): no gearing param, computed from plant
  frc::sim::DCMotorSim m_feedSim;
  frc::sim::DCMotorSim m_centerSim;

  frc::PIDController m_pid;

  // Control state
  enum class Mode { kVoltage, kVelocity, kStopped };
  Mode m_mode{Mode::kStopped};
  units::volt_t m_voltageCommand{0_V};
  units::turns_per_second_t m_velocitySetpoint{0_tps};

  // Ball simulation
  bool m_ballPresent{false};
  units::second_t m_ballTimer{0_s};
};
