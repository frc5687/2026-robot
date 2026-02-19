// Team 5687 2026

#pragma once

#include <array>

#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>

#include "Constants.h"
#include "IntakeIO.h"
#include "utils/CANDevice.h"

class CTREIntakeIO : public IntakeIO {
public:
  CTREIntakeIO(const CANDevice &leader, const CANDevice &follower);

  void UpdateInputs(IntakeIOInputs &inputs) override;
  void SetVoltage(units::volt_t voltage) override;
  void Stop() override;

private:
  ctre::phoenix6::hardware::TalonFX m_leader;
  ctre::phoenix6::hardware::TalonFX m_follower;

  // Leader signals only — follower mirrors output
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> &m_velocitySignal;
  ctre::phoenix6::StatusSignal<units::volt_t> &m_voltageSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_statorSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_supplySignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_followerStatorSignal;

  std::array<ctre::phoenix6::BaseStatusSignal *, 3> m_criticalSignals;
  std::array<ctre::phoenix6::BaseStatusSignal *, 2> m_batchedSignals;

  ctre::phoenix6::controls::VoltageOut m_voltageRequest{0_V};
  ctre::phoenix6::controls::NeutralOut m_neutralRequest{};

  void ConfigureDevices();
};
