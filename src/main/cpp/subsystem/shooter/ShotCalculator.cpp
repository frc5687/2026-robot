// Team 5687 2026

#include "subsystem/shooter/ShotCalculator.h"

#include <cmath>
#include <numbers>

#include "RobotState.h"
#include "subsystem/vision/FieldConstants.h"
#include "utils/Logger.h"

ShotCalculator::ShotCalculator() {
  // distance (m) → hood angle (deg)
  // 5 feet, 1200 rpm, 60 rps, 0 deg
  // 7 feet, 1300 rpm, 60 rps, 6 deg
  // 10 feet, 1400 rpm, 60 rps, 10 deg
  // 13 feet, 1520 rpm, 60 rps, 12 deg
  // 16 feet, 1650 rpm, 60 rps, 19 deg

  // 3/4/26

  std::vector<std::pair<double, double>> hoodMap{
      {1.443, 1.2},    {2.136, 5.25},    {2.959, 8.0}, {3.56, 11},
      {4.49, 15.5}, {5.0, 18}, {5.49, 18.0}};

  m_hoodAngleMap.InsertValues(hoodMap);

  // TODO: Make this use units

  std::vector<std::pair<double, double>> flyMap{
      {1.443, 1225},    {2.136, 1300},    {2.959, 1350}, {3.56, 1450},
      {4.49, 1600}, {5.0, 1690}, {5.49, 1650}};

  m_flywheelMap.InsertValues(flyMap);

  std::vector<std::pair<double, double>> tofMap{
      {1.443, 0.96},    {2.136, 0.97},    {2.959, 1.11}, {3.56, 1.23},
      {4.49, 1.22}, {5.0, 1.26}, {5.49, 1.5}};
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
  if (a < 0.0)
    a += 2.0 * std::numbers::pi;
  return units::radian_t{a - std::numbers::pi};
}

ShotSolution ShotCalculator::Calculate(units::second_t now, bool isRed) {
  auto &rs = RobotState::Instance();

  units::second_t launchTime = now + m_cfg.totalLatency;
  OdometryData futureDrive = rs.GetDriveState(launchTime);

  frc::Pose2d launcherPose =
      futureDrive.estimatedPose.TransformBy(m_cfg.robotToLauncher);
  Logger::Instance().Log("ShotCalculator/launcherPose", launcherPose);
  frc::Translation2d launcherXY = launcherPose.Translation();
  frc::Translation2d target = FlipAlliance(m_cfg.targetXY, isRed);
  Logger::Instance().Log("ShotCalculator/TargetPose", target);

  frc::Rotation2d heading = futureDrive.estimatedPose.Rotation();
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
  fieldVy += omega * offsetX_field;

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
Logger::Instance().Log(
      "ShotCalculator/ShotDistance", effDist);

  double hoodAngleDeg = m_hoodAngleMap.GetValue(effDist);
  double flywheelSpeed = m_flywheelMap.GetValue(effDist);
  tof = m_tofMap.GetValue(effDist);

  double aimX = tx - lx - fieldVx * tof;
  double aimY = ty - ly - fieldVy * tof;
  frc::Rotation2d aimToTarget{units::radian_t{std::atan2(aimY, aimX)}};
  frc::Rotation2d driveAngle = aimToTarget - m_cfg.robotToLauncher.Rotation();
  Logger::Instance().Log(
      "ShotCalculator/DrivePoseShot",
      frc::Pose2d{{units::meter_t{lookaheadX}, units::meter_t{lookaheadY}},
                  driveAngle});

  // Compare current robot heading against required drive angle.
  OdometryData currentDrive = rs.GetDriveState(now);
  bool driveOK =
      units::math::abs(NormalizeAngle(
          (currentDrive.estimatedPose.Rotation() - driveAngle).Radians())) <
      m_cfg.driveAngleTolerance;

  HoodState hood = rs.GetHoodState(now);
  FlywheelState flywheel = rs.GetFlywheelState(now);

  bool hoodOK = units::math::abs(hood.angle - units::degree_t{hoodAngleDeg}) <
                m_cfg.hoodTolerance;

  double flywheelRPM =
      flywheel.velocity.value() * 60.0 / (2.0 * std::numbers::pi);
  bool flywheelOK =
      flywheelSpeed > 0.0 &&
      std::abs(flywheelRPM - flywheelSpeed) / std::max(flywheelSpeed, 0.1) <
          m_cfg.flywheelToleranceFrac;

  bool inRange =
      effDist >= m_cfg.minDistanceMeters && effDist <= m_cfg.maxDistanceMeters;
  Logger::Instance().Log("ShotCalculator/driveAngle", driveAngle);
  Logger::Instance().Log("ShotCalculator/driveOK", driveOK);
  Logger::Instance().Log("ShotCalculator/hoodOK", hoodOK);
  Logger::Instance().Log("ShotCalculator/flywheelOK", flywheelOK);
  Logger::Instance().Log("ShotCalculator/inRange", inRange);

  return {
      .inRange = inRange,
      .ready = inRange && driveOK && hoodOK && flywheelOK,
      .driveAngle = driveAngle,
      .hoodAngle = units::degree_t{hoodAngleDeg},
      .flywheelSpeed = flywheelSpeed,
      .effectiveDistance = units::meter_t{effDist},
      .timeOfFlight = units::second_t{tof},
  };
}
