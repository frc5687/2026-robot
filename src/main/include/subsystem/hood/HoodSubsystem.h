// Team 5687 2026

#pragma once

#include "HoodIO.h"
#include "HoodState.h"
#include "subsystem/LoggedSubsystem.h"
#include "units/angle.h"
#include <unordered_map>

class HoodSubsystem : public LoggedSubsystem {

public:
  explicit HoodSubsystem(std::unique_ptr<HoodIO> io);
  ~HoodSubsystem() = default;
  void SetHoodPosition(units::turn_t desiredAngle);
  void SetHoodPosition(units::turn_t leftAngle, units::turn_t rightAngle);
  void SetMicroseconds(double microseconds);
  HoodState GetHoodState() const;
  void SetMicrosecondMap(std::vector<std::pair<double, int>>leftMap, std::vector<std::pair<double, int>>rightMap);
protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  std::unique_ptr<HoodIO> m_io;
  HoodIOInputs m_inputs{};
  units::angle::turn_t m_hoodRotation{0_tr};
  units::turn_t m_desiredAngle{0_tr};
};
