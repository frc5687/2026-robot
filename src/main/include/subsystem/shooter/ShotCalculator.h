// Team 5687 2026
#pragma once

#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include "utils/InterpolatingTreeMap.h"

struct ShotConfig {
  frc::Transform2d robotToLauncher{};
  frc::Translation2d targetXY{8.2296_m, 4.1148_m};

  units::second_t totalLatency{0.06_s};

  int maxAimIterations = 5;
  double convergenceThresholdMeters = 0.003;

  units::radian_t turretTolerance{0.035_rad};   // ~2°
  units::radian_t hoodTolerance{0.017_rad};     // ~1°
  double flywheelToleranceFrac = 0.03;          // 3%

  double minDistanceMeters = 1.34;
  double maxDistanceMeters = 5.60;
};

struct ShotSolution {
  bool inRange = false;
  bool ready = false;

  frc::Rotation2d turretFieldAngle{};
  units::radian_t turretRobotAngle{0};

  units::radian_t hoodAngle{0};
  double flywheelSpeed = 0;  // same unit as your flywheel map

  units::meter_t effectiveDistance{0};
  units::second_t timeOfFlight{0};
};

class ShotCalculator {
 public:
  ShotCalculator();

  ShotSolution Calculate(units::second_t now, bool isRed);

  InterpolatingDoubleTreeMap& HoodMap() { return m_hoodAngleMap; }
  InterpolatingDoubleTreeMap& FlywheelMap() { return m_flywheelMap; }
  InterpolatingDoubleTreeMap& TOFMap() { return m_tofMap; }
  void SetConfig(const ShotConfig& cfg) { m_cfg = cfg; }

 private:
  ShotConfig m_cfg;

  InterpolatingDoubleTreeMap m_hoodAngleMap;
  InterpolatingDoubleTreeMap m_flywheelMap;
  InterpolatingDoubleTreeMap m_tofMap;

  frc::Translation2d FlipAlliance(frc::Translation2d t, bool isRed) const;
};
