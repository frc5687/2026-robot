// Team 5687 2026

#pragma once

#include <units/angular_velocity.h>
#include <units/voltage.h>

#include <array>

#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>

#include "Constants.h"
#include "FlywheelIO.h"
#include "utils/CANDevice.h"

class CTREFlywheelIO : public FlywheelIO {
public:
  CTREFlywheelIO(const CANDevice &leader, const CANDevice &follower1,
                 const CANDevice &follower2, const CANDevice &follower3);
  ~CTREFlywheelIO() override = default;

  void UpdateInputs(FlywheelIOInputs &inputs) override;
  void SetMotorVelocity(units::turns_per_second_t rps) override;
  void SetVoltage(units::volt_t voltage) override;

private:
  ctre::phoenix6::hardware::TalonFX m_leader;
  ctre::phoenix6::hardware::TalonFX m_follower1;
  ctre::phoenix6::hardware::TalonFX m_follower2;
  ctre::phoenix6::hardware::TalonFX m_follower3;

  ctre::phoenix6::StatusSignal<units::turns_per_second_t>
      &m_leaderVelocitySignal;
  ctre::phoenix6::StatusSignal<units::turn_t> &m_leaderPositionSignal;
  ctre::phoenix6::StatusSignal<units::volt_t> &m_leaderVoltageSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_leaderStatorCurrentSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_leaderSupplyCurrentSignal;

  ctre::phoenix6::StatusSignal<units::ampere_t> &m_follower1StatorCurrentSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_follower2StatorCurrentSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_follower3StatorCurrentSignal;

  std::array<ctre::phoenix6::BaseStatusSignal *, 5> m_criticalSignals;
  std::array<ctre::phoenix6::BaseStatusSignal *, 3> m_diagnosticSignals;

  ctre::phoenix6::controls::VelocityVoltage m_velocityRequest{0_tps};
  ctre::phoenix6::controls::VoltageOut m_voltageRequest{0_V};
  ctre::phoenix6::controls::NeutralOut m_neutralRequest{};

  ctre::phoenix6::configs::TalonFXConfiguration m_leaderConfig{};
  ctre::phoenix6::configs::TalonFXConfiguration m_followerConfig{};

  bool m_characterizing{false};
  units::volt_t m_characterizationVoltage{0_V};
  units::turns_per_second_t m_setpoint{0_tps};

  void ConfigureDevices();
  void ConfigureLeader(ctre::phoenix6::configs::TalonFXConfiguration &config,
                       bool inverted);
  void ConfigureFollower(ctre::phoenix6::hardware::TalonFX &follower,
                         int leaderDeviceId, bool opposeLeader);
  void ConfigureClosedLoop();
  void ConfigureSignalFrequencies();
};
