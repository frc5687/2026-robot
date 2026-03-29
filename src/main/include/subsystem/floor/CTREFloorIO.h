// Team 5687 2026

#pragma once

#include <array>

#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>

#include "FloorIO.h"
#include "utils/CANDevice.h"

class CTREFloorIO : public FloorIO {
public:
  explicit CTREFloorIO(const CANDevice &motor);

  void UpdateInputs(FloorIOInputs &inputs) override;
  void SetVoltage(units::volt_t voltage) override;
  void SetVelocity(units::turns_per_second_t rps) override;
  void Stop() override;

private:
  ctre::phoenix6::hardware::TalonFX m_motor;

  ctre::phoenix6::StatusSignal<units::turn_t> &m_positionSignal;
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> &m_velocitySignal;
  ctre::phoenix6::StatusSignal<units::volt_t> &m_voltageSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_statorSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_supplySignal;

  std::array<ctre::phoenix6::BaseStatusSignal *, 5> m_signals;

  ctre::phoenix6::controls::VelocityVoltage m_velocityRequest{0_tps};
  ctre::phoenix6::controls::VoltageOut m_voltageRequest{0_V};
  ctre::phoenix6::controls::NeutralOut m_neutralRequest{};

  ctre::phoenix6::configs::TalonFXConfiguration m_config{};

  void ConfigureDevices();
  void ConfigureClosedLoop();
  void ConfigureSignalFrequencies();
};
