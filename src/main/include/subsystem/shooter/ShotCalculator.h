// Team 5687 2026

#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>

#include "Constants.h"
#include "utils/InterpolatingTreeMap.h"

struct ShotConfig {
  //frc::Transform2d robotToLauncher{{-0.114_m, 0.191_m}, frc::Rotation2d{0_rad}};
  frc::Transform2d robotToLauncher{{0.0_m, 0.191_m}, frc::Rotation2d{0_rad}}; // We are assuming to aim from between turrets
  frc::Translation2d targetXY{
      Constants::Field::Hub::kInnerCenterPoint.ToTranslation2d()};
  units::second_t totalLatency{0.06_s};
  int maxAimIterations = 5;
  double convergenceThresholdMeters = 0.003;

  units::radian_t turretTolerance{0.035_rad}; // ~2 deg
  units::radian_t hoodTolerance{0.017_rad};   // ~1 deg
  double flywheelToleranceFrac = 0.03;        // 3 %

  double minDistanceMeters = 1.34;
  double maxDistanceMeters = 5.60;
};

struct ShotSolution {
  bool inRange = false;
  bool ready = false;

  frc::Rotation2d turretFieldAngle{};
  units::radian_t turretRobotAngle{0};

  units::degree_t hoodAngle{0};
  double flywheelSpeed = 0;

  units::meter_t effectiveDistance{0};
  units::second_t timeOfFlight{0};
};

class ShotCalculator {
public:
  using InterpolatingDoubleTreeMap = InterpolatingTreeMap<double, double>;
  ShotCalculator();

  ShotSolution Calculate(units::second_t now, bool isRed);

  InterpolatingDoubleTreeMap &HoodMap() { return m_hoodAngleMap; }
  InterpolatingDoubleTreeMap &FlywheelMap() { return m_flywheelMap; }
  InterpolatingDoubleTreeMap &TOFMap() { return m_tofMap; }
  void SetConfig(const ShotConfig &cfg) { m_cfg = cfg; }
  ShotConfig GetConfig() const { return m_cfg; };

private:
  ShotConfig m_cfg;

  InterpolatingDoubleTreeMap m_hoodAngleMap; // distance_m → degrees
  InterpolatingDoubleTreeMap m_flywheelMap;  // distance_m → flywheel units
  InterpolatingDoubleTreeMap m_tofMap;       // distance_m → seconds

  frc::Translation2d FlipAlliance(frc::Translation2d t, bool isRed) const;

  static units::radian_t NormalizeAngle(units::radian_t angle);
};
