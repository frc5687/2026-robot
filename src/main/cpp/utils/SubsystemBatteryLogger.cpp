// Team 5687 2026

#include "utils/SubsystemBatteryLogger.h"
#include <frc/RobotController.h>
#include <units/math.h>

void SubsystemBatteryLogger::RegisterSubsystem(std::string name,
                                               CurrentProvider currentProvider,
                                               PowerProvider powerProvider) {
  m_subsystems.push_back(
      TrackedSubsystem{std::move(name), std::move(currentProvider),
                       std::move(powerProvider), units::joule_t{0.0}});
}

void SubsystemBatteryLogger::RegisterAggregateGroup(
    std::string name, CurrentProvider currentProvider,
    PowerProvider powerProvider) {
  m_aggregateGroups.push_back(
      TrackedSubsystem{std::move(name), std::move(currentProvider),
                       std::move(powerProvider), units::joule_t{0.0}});
}

void SubsystemBatteryLogger::Update() {
  const units::second_t now{frc::Timer::GetFPGATimestamp()};
  if (m_hasLastTimestamp) {
    const units::second_t deltaTime = now - m_lastTimestamp;

    if (deltaTime > units::second_t{0.0}) {
      units::ampere_t totalCurrent{0.0};
      units::watt_t totalPower{0.0};

      for (auto &subsystem : m_subsystems) {
        const auto current = units::math::abs(subsystem.currentProvider());
        const auto power = units::math::abs(subsystem.powerProvider());
        const units::joule_t energyIncrement = power * deltaTime;

        subsystem.accumulatedEnergy += energyIncrement;
        m_totalEnergy += energyIncrement;

        totalCurrent += current;
        totalPower += power;

        const units::watt_hour_t accumulatedWh = subsystem.accumulatedEnergy;

        Logger::Instance().Log("Power/" + subsystem.name + "/Current",
                               current.value());
        Logger::Instance().Log("Power/" + subsystem.name + "/Power",
                               power.value());
        Logger::Instance().Log("Power/" + subsystem.name + "/EnergyWattHours",
                               accumulatedWh.value());
      }

      for (auto &group : m_aggregateGroups) {
        const auto current = units::math::abs(group.currentProvider());
        const auto power = units::math::abs(group.powerProvider());
        const units::joule_t energyIncrement = power * deltaTime;

        group.accumulatedEnergy += energyIncrement;

        const units::watt_hour_t accumulatedWh = group.accumulatedEnergy;

        Logger::Instance().Log("Power/" + group.name + "/Current",
                               current.value());
        Logger::Instance().Log("Power/" + group.name + "/Power", power.value());
        Logger::Instance().Log("Power/" + group.name + "/EnergyWattHours",
                               accumulatedWh.value());
      }

      const units::volt_t batteryVoltage =
          frc::RobotController::GetBatteryVoltage();
      const units::volt_t inputVoltage{
          frc::RobotController::GetInputVoltage()};
      const units::ampere_t inputCurrent{
          frc::RobotController::GetInputCurrent()};
      const units::watt_t inputPower = inputVoltage * inputCurrent;

      auto &logger = Logger::Instance();

      logger.Log("Power/RIO/BatteryVoltage", batteryVoltage.value());
      logger.Log("Power/RIO/InputVoltage", inputVoltage.value());
      logger.Log("Power/RIO/InputCurrent", inputCurrent.value());
      logger.Log("Power/RIO/InputPower", inputPower.value());

      const units::watt_hour_t totalWh = m_totalEnergy;

      logger.Log("Power/Total/Current", totalCurrent.value());
      logger.Log("Power/Total/Power", totalPower.value());
      logger.Log("Power/Total/EnergyWattHours", totalWh.value());
    }
  }

  m_lastTimestamp = now;
  m_hasLastTimestamp = true;
}
