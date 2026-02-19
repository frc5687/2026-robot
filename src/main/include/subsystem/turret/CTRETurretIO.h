// Team 5687 2026

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>

#include "subsystem/turret/TurretIO.h"
#include "utils/CANDevice.h"

class CTRETurretIO : public TurretIO {
public:
  explicit CTRETurretIO(const CANDevice &id);
  void UpdateInputs(TurretIOInputs &inputs) override;
  void SetTurretAngle(units::radian_t angle) override;
  void Stop();
  void SetBrakeMode(bool brake);
  void ZeroPosition();

private:
  ctre::phoenix6::hardware::TalonFX m_motor;

  ctre::phoenix6::configs::TalonFXConfiguration m_motorConfig{};

  // Control requests
  ctre::phoenix6::controls::MotionMagicVoltage m_positionRequest{0_tr};
  ctre::phoenix6::controls::VoltageOut m_voltageRequest{0_V};

  // Status signals
  ctre::phoenix6::StatusSignal<units::turn_t> &m_positionSignal;
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> &m_velocitySignal;
  ctre::phoenix6::StatusSignal<units::turns_per_second_squared_t>
      &m_accelerationSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_currentSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t> &m_torqueCurrentSignal;
  ctre::phoenix6::StatusSignal<units::celsius_t> &m_tempSignal;

  void ConfigureDevices();
  void ConfigureClosedLoop();
  void ConfigureSignalFrequencies();
};
