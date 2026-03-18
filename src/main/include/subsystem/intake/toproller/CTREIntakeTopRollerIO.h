// Team 5687 2026

#pragma once

#include <array>

#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>

#include "subsystem/intake/IntakeConstants.h"
#include "IntakeTopRollerIO.h"
#include "utils/CANDevice.h"

class CTREIntakeTopRollerIO : public IntakeTopRollerIO {
public:
  CTREIntakeTopRollerIO(const CANDevice &leader, const CANDevice &follower);

  void UpdateInputs(IntakeTopRollerIOInputs &inputs) override;
  void SetVoltage(units::volt_t voltage) override;
  void SetVelocity(units::turns_per_second_t rps) override;
  void Stop() override;

private:
  ctre::phoenix6::hardware::TalonFX m_leader;
  ctre::phoenix6::hardware::TalonFX m_follower;

  ctre::phoenix6::StatusSignal<units::turn_t> &m_positionSignal;
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> &m_velocitySignal;
  ctre::phoenix6::StatusSignal<units::volt_t> &m_voltageSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_statorSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_supplySignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_followerStatorSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_followerSupplySignal;
  ctre::phoenix6::StatusSignal<units::volt_t> &m_followerVoltageSignal;

  std::array<ctre::phoenix6::BaseStatusSignal *, 6> m_criticalSignals;
  std::array<ctre::phoenix6::BaseStatusSignal *, 2> m_batchedSignals;

  ctre::phoenix6::controls::VelocityVoltage m_velocityRequest{0_tps};
  ctre::phoenix6::controls::VoltageOut m_voltageRequest{0_V};
  ctre::phoenix6::controls::NeutralOut m_neutralRequest{};

  ctre::phoenix6::configs::TalonFXConfiguration m_leaderConfig{};

  void ConfigureDevices();
  void ConfigureClosedLoop();
  void ConfigureSignalFrequencies();
};
