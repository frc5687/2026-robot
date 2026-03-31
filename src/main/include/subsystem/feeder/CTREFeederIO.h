// Team 5687 2026

#pragma once

#include <array>

#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>

#include "FeederIO.h"
#include "ctre/phoenix6/CANrange.hpp"
#include "ctre/phoenix6/controls/TorqueCurrentFOC.hpp"
#include "ctre/phoenix6/core/CoreCANrange.hpp"
#include "utils/CANDevice.h"

class CTREFeederIO : public FeederIO {
public:
  CTREFeederIO(const CANDevice &leader, const CANDevice &follower, const CANDevice &canRange);

  void UpdateInputs(FeederIOInputs &inputs) override;
  void SetVoltage(units::volt_t voltage) override;
  void SetCurrent(units::ampere_t current) override;
  void SetVelocity(units::turns_per_second_t rps) override;
  void SetPosition(units::turn_t position) override;
  void Stop() override;

private:
  ctre::phoenix6::hardware::TalonFX m_leader;
  ctre::phoenix6::hardware::TalonFX m_follower;

  ctre::phoenix6::hardware::CANrange m_canRange;

  ctre::phoenix6::StatusSignal<units::turn_t> &m_positionSignal;
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> &m_velocitySignal;
  ctre::phoenix6::StatusSignal<units::volt_t> &m_voltageSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_statorSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_supplySignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_followerStatorSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_followerSupplySignal;
  ctre::phoenix6::StatusSignal<units::volt_t> &m_followerVoltageSignal;
  ctre::phoenix6::StatusSignal<bool> &m_fuelDetected;

  std::array<ctre::phoenix6::BaseStatusSignal *, 8> m_signals;

  ctre::phoenix6::controls::VelocityVoltage m_velocityRequest{0_tps};
  ctre::phoenix6::controls::PositionVoltage m_positionRequest{0_tr};
  ctre::phoenix6::controls::VoltageOut m_voltageRequest{0_V};
  ctre::phoenix6::controls::NeutralOut m_neutralRequest{};
  ctre::phoenix6::controls::TorqueCurrentFOC m_currentRequest{0_A};

  ctre::phoenix6::configs::TalonFXConfiguration m_leaderConfig{};

  ctre::phoenix6::configs::CANrangeConfiguration m_canRangeConfig{};

  void ConfigureDevices();
  void ConfigureCANRange();
  void ConfigureFollower(ctre::phoenix6::hardware::TalonFX &follower,
                        int leaderDeviceId, bool opposeLeader);
  void ConfigureClosedLoop();
  void ConfigureSignalFrequencies();
};
