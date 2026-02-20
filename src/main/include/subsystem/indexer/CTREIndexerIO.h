// Team 5687 2026

#pragma once

#include <array>

#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>

#include "Constants.h"
#include "IndexerIO.h"
#include "utils/CANDevice.h"

class CTREIndexerIO : public IndexerIO {
public:
  CTREIndexerIO(const CANDevice &leftMotor, const CANDevice &rightMotor,
                const CANDevice &centerMotor, const CANDevice& centerMotorFollower);

  void UpdateInputs(IndexerIOInputs &inputs) override;
  void SetVoltage(units::volt_t voltage) override;
  void SetMotorVelocity(units::turns_per_second_t rps) override;
  void Stop() override;

private:
  // ── Motors (left = leader, right = follower, center = independent) ──
  ctre::phoenix6::hardware::TalonFX m_leftMotor;
  ctre::phoenix6::hardware::TalonFX m_rightMotor;
  ctre::phoenix6::hardware::TalonFX m_centerMotor;
  ctre::phoenix6::hardware::TalonFX m_centerMotorFollower;

  // ── Status signals (leader + center) ─────────────────────────────────
  ctre::phoenix6::StatusSignal<units::turn_t> &m_positionSignal;
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> &m_velocitySignal;
  ctre::phoenix6::StatusSignal<units::volt_t> &m_voltageSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_statorSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_supplySignal;

  ctre::phoenix6::StatusSignal<units::turn_t> &m_centerPositionSignal;
  ctre::phoenix6::StatusSignal<units::turns_per_second_t>
      &m_centerVelocitySignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_centerStatorSignal;

  // ── Signal batching ──────────────────────────────────────────────────
  std::array<ctre::phoenix6::BaseStatusSignal *, 5> m_criticalSignals;
  std::array<ctre::phoenix6::BaseStatusSignal *, 3> m_batchedSignals;

  // ── Control requests ─────────────────────────────────────────────────
  ctre::phoenix6::controls::VelocityVoltage m_velocityRequest{0_tps};
  ctre::phoenix6::controls::VoltageOut m_voltageRequest{0_V};
  ctre::phoenix6::controls::NeutralOut m_neutralRequest{};

  // ── Configs ──────────────────────────────────────────────────────────
  ctre::phoenix6::configs::TalonFXConfiguration m_leftConfig{};
  ctre::phoenix6::configs::TalonFXConfiguration m_centerConfig{};

  // ── Configuration helpers ────────────────────────────────────────────
  void ConfigureDevices();
  void ConfigureClosedLoop();
  void ConfigureSignalFrequencies();
};
