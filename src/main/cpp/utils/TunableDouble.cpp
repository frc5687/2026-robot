// Team 5687 2026

#include "utils/TunableDouble.h"

#include <string>

void TunableDouble::Init(nt::DoubleTopic topic, double defaultValue) {
  m_topic = topic;
  m_defaultValue = defaultValue;

  m_entry = m_topic.GetEntry(m_defaultValue);

  m_entry.SetDefault(m_defaultValue);
  m_cached = m_entry.Get(m_defaultValue);
  m_lastChange = m_entry.GetLastChange();
}

TunableDouble::TunableDouble(const std::string &table, const std::string &topic,
                             double defaultValue) {
  m_table = nt::NetworkTableInstance::GetDefault().GetTable(table);
  Init(m_table->GetDoubleTopic(topic), defaultValue);
}

TunableDouble::TunableDouble(const std::string &fullTopicPath,
                             double defaultValue) {
  auto inst = nt::NetworkTableInstance::GetDefault();
  Init(inst.GetDoubleTopic(fullTopicPath), defaultValue);
}

bool TunableDouble::HasChanged() {
  double current = m_entry.Get(m_defaultValue);
  if (current == m_cached)
    return false;

  m_cached = current;
  m_lastChange = m_entry.GetLastChange();
  return true;
}
