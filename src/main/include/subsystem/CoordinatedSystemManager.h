// Team 5687 2026

#pragma once

#include <iostream>
#include <vector>

#include "subsystem/shooter/CoordinatedSystem.h"

class CoordinatedSystemManager {
public:
  static CoordinatedSystemManager &Instance() {
    static CoordinatedSystemManager inst;
    return inst;
  }

  void AddSystem(CoordinatedSystem *system) {
    for (CoordinatedSystem *storedSystem : m_systems) {
      if (storedSystem == system) {
        std::cerr << "System ptr already in Manager\n";
        return;
      }
    }
    m_systems.push_back(system);
  }

  void UpdateAll() {
    for (CoordinatedSystem *system : m_systems) {
      system->Update();
    }
  }

private:
  std::vector<CoordinatedSystem *> m_systems;
};
