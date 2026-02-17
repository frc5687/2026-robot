#include "subsystem/shooter/CoordinatedSystem.h"
#include "subsystem/CoordinatedSystemManager.h"
#include <frc/Timer.h>

CoordinatedSystem::CoordinatedSystem(const std::string& name)
    : m_name(name),
      m_table(nt::NetworkTableInstance::GetDefault().GetTable(name)) {
    CoordinatedSystemManager::Instance().AddSystem(this);
}

void CoordinatedSystem::Periodic() {
  auto startTime = frc::Timer::GetFPGATimestamp();

  // Always update inputs and telemetry
  Update();
  LogTelemetry();

  auto avgUpdateTime =
      m_updateCount > 0 ? m_totalUpdateTime / m_updateCount : 0_s;

  Log("Performance/AverageUpdateTime", avgUpdateTime.value() * 1000.0);  // ms
  Log("Performance/UpdateCount", static_cast<double>(m_updateCount));

  auto endTime = frc::Timer::GetFPGATimestamp();
  auto updateTime = (endTime - startTime);
  m_totalUpdateTime += updateTime;
  Log("Performance/UpdateTime", updateTime.value() * 1000);
  m_updateCount++;
}
