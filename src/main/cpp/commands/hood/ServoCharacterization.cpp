// Team 5687 2026

#include "commands/hood/ServoCharacterization.h"
#include "frc/Timer.h"
#include "subsystem/hood/HoodState.h"
#include "subsystem/hood/HoodSubsystem.h"
#include "subsystem/shooter/ShotCalculator.h"
#include <algorithm>
#include <utility>

ServoCharacterization::ServoCharacterization(HoodSubsystem *hood)
    : m_hoodSubsystem(hood) {
  AddRequirements(hood);
}

void ServoCharacterization::Initialize() {
    m_hoodSubsystem->SetMicroseconds(m_targetMicroseconds);
    m_lastMeasurement = frc::Timer::GetFPGATimestamp();
}

void ServoCharacterization::Execute() {
  m_dt = m_lastMeasurement - frc::Timer::GetFPGATimestamp();
  if(m_dt >= 0.75_s){
    HoodState currentState = m_hoodSubsystem->GetHoodState();
    m_leftMicrosecondMap.emplace_back(std::make_pair(currentState.angle, m_targetMicroseconds));//TODO: redo when we have seperated angles
    m_rightMicrosecondMap.emplace_back(std::make_pair(currentState.angle, m_targetMicroseconds));//TODO: redo when we have seperated angles
    m_targetMicroseconds += 50;
    m_lastMeasurement = frc::Timer::GetFPGATimestamp();
  }
}

void ServoCharacterization::End(bool interrupted) {
  m_hoodSubsystem->SetMicrosecondMap(m_rightMicrosecondMap, m_rightMicrosecondMap);
}


bool ServoCharacterization::IsFinished() {
  if (m_targetMicroseconds >= 2500) {
    return true;
  }
}