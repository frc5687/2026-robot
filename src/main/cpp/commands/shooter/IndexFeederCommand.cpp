// Team 5687 2026

#include "commands/shooter/IndexFeederCommand.h"

#include <units/math.h>

IndexFeederCommand::IndexFeederCommand(FeederSubsystem *feeder)
    : m_feeder(feeder) {
  AddRequirements(m_feeder);
  SetName("IndexFeederCommand");
}

void IndexFeederCommand::Initialize() {
  if (m_feeder->isFuelDetected()) {
    m_state = State::Done;
  } else {
    m_state = State::Feeding;
  }
  m_currentCycle = 0;
}

void IndexFeederCommand::Execute() {
  switch (m_state) {
  case State::Feeding:
    m_feeder->SetVoltage(kFeederFeedVoltage);

    if (m_feeder->isFuelDetected()) {
      m_currentCycle = 0;
      m_jogStartPosition = m_feeder->GetPosition();
      m_feeder->SetPosition(m_jogStartPosition + kJogForwardRotations);
      m_state = State::JogForward;
    }
    break;

  case State::JogForward: {
    auto traveled =
        units::math::abs(m_feeder->GetPosition() - m_jogStartPosition);
    if (traveled >= kJogForwardRotations - 0.25_tr) {
      bool isFinalCycle = (m_currentCycle + 1 >= kIndexCycles);
      auto backwardDist = isFinalCycle ? kFinalJogBackwardRotations : kJogBackwardRotations;
      m_jogStartPosition = m_feeder->GetPosition();
      m_feeder->SetPosition(m_jogStartPosition - backwardDist);
      m_state = State::JogBackward;
    }
    break;
  }

  case State::JogBackward: {
    bool isFinalCycle = (m_currentCycle + 1 >= kIndexCycles);
    auto targetDist = isFinalCycle ? kFinalJogBackwardRotations : kJogBackwardRotations;
    auto traveled =
        units::math::abs(m_feeder->GetPosition() - m_jogStartPosition);
    if (traveled >= targetDist - 0.25_tr) {
      m_currentCycle++;
      if (m_currentCycle >= kIndexCycles) {
        m_state = State::Done;
      } else {
        m_jogStartPosition = m_feeder->GetPosition();
        m_feeder->SetPosition(m_jogStartPosition + kJogForwardRotations);
        m_state = State::JogForward;
      }
    }
    break;
  }

  case State::Done:
    break;
  }
}

void IndexFeederCommand::End(bool interrupted) {
  m_feeder->Stop();
  if (!interrupted) {
    m_feeder->SetIndexed();
  }
}

bool IndexFeederCommand::IsFinished() { return m_state == State::Done; }
