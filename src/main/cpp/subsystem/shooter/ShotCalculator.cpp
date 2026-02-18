// Team 5687 2026
#include "subsystem/shooter/ShotCalculator.h"

#include <cmath>
#include <numbers>

#include "Constants.h"
#include "RobotState.h"
#include "utils/Logger.h"

ShotCalculator::ShotCalculator() {
  // distance (m) → hood angle (deg)
  std::vector<std::pair<double, double>> hoodMap{
      {1.34, 19.0}, {1.78, 19.0}, {2.17, 24.0}, {2.81, 27.0}, {3.82, 29.0},
      {4.09, 30.0}, {4.40, 31.0}, {4.77, 32.0}, {5.57, 32.0}, {5.60, 35.0}};
  m_hoodAngleMap.InsertValues(hoodMap);

  // distance (m) → flywheel speed
  std::vector<std::pair<double, double>> flyMap{
      {1.34, 210.0}, {1.78, 220.0}, {2.17, 220.0}, {2.81, 230.0},
      {3.82, 250.0}, {4.09, 255.0}, {4.40, 260.0}, {4.77, 265.0},
      {5.57, 275.0}, {5.60, 290.0}};
  m_flywheelMap.InsertValues(flyMap);

  // distance (m) → time of flight (s)
  std::vector<std::pair<double, double>> tofMap{
      {1.38, 0.90}, {1.88, 1.09}, {3.15, 1.11}, {4.55, 1.12}, {5.68, 1.16}};
  m_tofMap.InsertValues(tofMap);
}

frc::Translation2d ShotCalculator::FlipAlliance(frc::Translation2d t,
                                                 bool isRed) const {
  if (isRed) {
    return {units::meter_t{Constants::Field::kFieldLength} - t.X(), t.Y()};
  }
  return t;
}

units::radian_t ShotCalculator::NormalizeAngle(units::radian_t angle) {
  double a = angle.value();
  a = std::fmod(a + std::numbers::pi, 2.0 * std::numbers::pi);
  if (a < 0.0) a += 2.0 * std::numbers::pi;
  return units::radian_t{a - std::numbers::pi};
}

ShotSolution ShotCalculator::Calculate(units::second_t now, bool isRed) {
  auto& rs = RobotState::Instance();

  units::second_t launchTime = now + m_cfg.totalLatency;
  OdometryData futureDrive = rs.GetDriveState(launchTime);

  frc::Pose2d launcherPose =
      futureDrive.pose.TransformBy(m_cfg.robotToLauncher);
  Logger::Instance().Log("ShotCalculator/launcherPose", launcherPose);
  frc::Translation2d launcherXY = launcherPose.Translation();
  frc::Translation2d target = FlipAlliance(m_cfg.targetXY, isRed);

  frc::Rotation2d heading = futureDrive.pose.Rotation();
  double cos_h = heading.Cos();
  double sin_h = heading.Sin();

  double rvx = futureDrive.chassisSpeeds.vx.value();
  double rvy = futureDrive.chassisSpeeds.vy.value();
  double omega = futureDrive.chassisSpeeds.omega.value();

  double fieldVx = rvx * cos_h - rvy * sin_h;
  double fieldVy = rvx * sin_h + rvy * cos_h;

  double offsetX_robot = m_cfg.robotToLauncher.X().value();
  double offsetY_robot = m_cfg.robotToLauncher.Y().value();
  double offsetX_field = offsetX_robot * cos_h - offsetY_robot * sin_h;
  double offsetY_field = offsetX_robot * sin_h + offsetY_robot * cos_h;

  fieldVx += -omega * offsetY_field;
  fieldVy +=  omega * offsetX_field;

  double lx = launcherXY.X().value();
  double ly = launcherXY.Y().value();
  double tx = target.X().value();
  double ty = target.Y().value();

  double lookaheadX = lx;
  double lookaheadY = ly;
  double effDist = std::hypot(tx - lookaheadX, ty - lookaheadY);
  double tof = m_tofMap.GetValue(effDist);

  for (int i = 0; i < m_cfg.maxAimIterations; ++i) {
    double newLookaheadX = lx + fieldVx * tof;
    double newLookaheadY = ly + fieldVy * tof;
    double newDist = std::hypot(tx - newLookaheadX, ty - newLookaheadY);

    lookaheadX = newLookaheadX;
    lookaheadY = newLookaheadY;

    if (std::abs(newDist - effDist) < m_cfg.convergenceThresholdMeters) {
      effDist = newDist;
      break;
    }
    effDist = newDist;
    tof = m_tofMap.GetValue(effDist);
  }

  double hoodAngleDeg = m_hoodAngleMap.GetValue(effDist);
  double flywheelSpeed = m_flywheelMap.GetValue(effDist);
  tof = m_tofMap.GetValue(effDist);

  frc::Rotation2d turretFieldAngle{
      units::radian_t{std::atan2(ty - lookaheadY, tx - lookaheadX)}};

  units::radian_t turretRobotAngle =
    NormalizeAngle((turretFieldAngle - heading).Radians());

  TurretState turret = rs.GetTurretState(now);
  HoodState hood = rs.GetHoodState(now);
  FlywheelState flywheel = rs.GetFlywheelState(now);

  bool turretOK =
      units::math::abs(NormalizeAngle(turret.angle - turretRobotAngle)) <
      m_cfg.turretTolerance;

  bool hoodOK =
      units::math::abs(hood.angle - units::degree_t{hoodAngleDeg}) <
      m_cfg.hoodTolerance;

  bool flywheelOK =
      flywheelSpeed > 0.0 &&
      std::abs(flywheel.velocity.value() - flywheelSpeed) /
              std::max(flywheelSpeed, 0.1) <
          m_cfg.flywheelToleranceFrac;

  bool inRange = effDist >= m_cfg.minDistanceMeters &&
                 effDist <= m_cfg.maxDistanceMeters;

  return {
      .inRange = inRange,
      .ready = inRange && turretOK && hoodOK && flywheelOK,
      .turretFieldAngle = turretFieldAngle,
      .turretRobotAngle = turretRobotAngle,
      .hoodAngle = units::degree_t{hoodAngleDeg},
      .flywheelSpeed = flywheelSpeed,
      .effectiveDistance = units::meter_t{effDist},
      .timeOfFlight = units::second_t{tof},
  };
}
