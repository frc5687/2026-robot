// Team 5687 2026

#pragma once

#include <array>

#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>

#include "FeederIO.h"
#include "utils/CANDevice.h"

class CTREFeederIO : public FeederIO {
public:
  CTREFeederIO(const CANDevice &leader, const CANDevice &follower1,
               const CANDevice &follower2, const CANDevice &follower3);

  void UpdateInputs(FeederIOInputs &inputs) override;
  void SetVoltage(units::volt_t voltage) override;
  void SetVelocity(units::turns_per_second_t rps) override;
  void Stop() override;

private:
  ctre::phoenix6::hardware::TalonFX m_leader;
  ctre::phoenix6::hardware::TalonFX m_follower1;
  ctre::phoenix6::hardware::TalonFX m_follower2;
  ctre::phoenix6::hardware::TalonFX m_follower3;

  ctre::phoenix6::StatusSignal<units::turn_t> &m_positionSignal;
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> &m_velocitySignal;
  ctre::phoenix6::StatusSignal<units::volt_t> &m_voltageSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_statorSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_supplySignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_follower1StatorSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_follower1SupplySignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_follower2StatorSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_follower2SupplySignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_follower3StatorSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_follower3SupplySignal;
  ctre::phoenix6::StatusSignal<units::volt_t> &m_follower1VoltageSignal;
  ctre::phoenix6::StatusSignal<units::volt_t> &m_follower2VoltageSignal;
  ctre::phoenix6::StatusSignal<units::volt_t> &m_follower3VoltageSignal;

  std::array<ctre::phoenix6::BaseStatusSignal *, 10> m_criticalSignals;
  std::array<ctre::phoenix6::BaseStatusSignal *, 4> m_batchedSignals;

  ctre::phoenix6::controls::VelocityVoltage m_velocityRequest{0_tps};
  ctre::phoenix6::controls::VoltageOut m_voltageRequest{0_V};
  ctre::phoenix6::controls::NeutralOut m_neutralRequest{};

  ctre::phoenix6::configs::TalonFXConfiguration m_leaderConfig{};

  void ConfigureDevices();
  void ConfigureFollower(ctre::phoenix6::hardware::TalonFX &follower,
                        int leaderDeviceId, bool opposeLeader);
  void ConfigureClosedLoop();
  void ConfigureSignalFrequencies();
};
