// Team 5687 2026

#pragma once

#include <units/angular_velocity.h>
#include <units/voltage.h>

#include <array>
#include <string>

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
  CTREFlywheelIO(const CANDevice &leftLeader, const CANDevice &leftFollower,
                 const CANDevice &rightFollower, const CANDevice &rightFollowerBottom);
  ~CTREFlywheelIO() override = default;

  void UpdateInputs(FlywheelIOInputs &inputs) override;
  void SetMotorVelocity(units::turns_per_second_t leftRPS,
                        units::turns_per_second_t rightRPS) override;
  void SetVoltage(units::volt_t voltage) override;

private:
  ctre::phoenix6::hardware::TalonFX m_leftLeader;
  ctre::phoenix6::hardware::TalonFX m_leftFollower;
  ctre::phoenix6::hardware::TalonFX m_rightFollower;
  ctre::phoenix6::hardware::TalonFX m_rightFollowerBottom;

  ctre::phoenix6::StatusSignal<units::turns_per_second_t> &m_leftVelocitySignal;
  ctre::phoenix6::StatusSignal<units::turn_t> &m_leftPositionSignal;
  ctre::phoenix6::StatusSignal<units::volt_t> &m_leftVoltageSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_leftStatorCurrentSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_leftSupplyCurrentSignal;

  ctre::phoenix6::StatusSignal<units::turns_per_second_t>
      &m_rightVelocitySignal;
  ctre::phoenix6::StatusSignal<units::turn_t> &m_rightPositionSignal;
  ctre::phoenix6::StatusSignal<units::volt_t> &m_rightVoltageSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_rightStatorCurrentSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_rightSupplyCurrentSignal;

  ctre::phoenix6::StatusSignal<units::ampere_t>
      &m_leftFollowerStatorCurrentSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t>
      &m_rightFollowerStatorCurrentSignal;

  std::array<ctre::phoenix6::BaseStatusSignal *, 6> m_criticalSignals;
  std::array<ctre::phoenix6::BaseStatusSignal *, 6> m_batchedSignals;

  ctre::phoenix6::controls::VelocityVoltage m_velocityRequest{0_tps};
  ctre::phoenix6::controls::VoltageOut m_voltageRequest{0_V};
  ctre::phoenix6::controls::NeutralOut m_neutralRequest{};

  ctre::phoenix6::configs::TalonFXConfiguration m_leftLeaderConfig{};
  ctre::phoenix6::configs::TalonFXConfiguration m_followerConfig{};

  bool m_characterizing{false};
  units::volt_t m_characterizationVoltage{0_V};
  units::turns_per_second_t m_leftSetpoint{0_tps};
  units::turns_per_second_t m_rightSetpoint{0_tps};

  void ConfigureDevices();
  void ConfigureLeader(ctre::phoenix6::configs::TalonFXConfiguration &config,
                       bool inverted);
  void ConfigureFollower(ctre::phoenix6::hardware::TalonFX &follower,
                         int leaderDeviceId, bool opposeLeader);
  void ConfigureClosedLoop();
  void ConfigureSignalFrequencies();
};
