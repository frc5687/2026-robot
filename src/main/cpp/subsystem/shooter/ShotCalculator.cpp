// Team 5687 2026
#include "subsystem/shooter/ShotCalculator.h"

#include <cmath>
#include "Constants.h"
#include "RobotState.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include <iostream>

ShotCalculator::ShotCalculator() {
    // THese are fake, 6328s
    std::vector<std::pair<double, double>> hoodMap{
        {1.34, 19.0},
        {1.78, 19.0},
        {2.17, 24.0},
        {2.81, 27.0},
        {3.82, 29.0},
        {4.09, 30.0},
        {4.40, 31.0},
        {4.77, 32.0},
        {5.57, 32.0},
        {5.60, 35.0}};
    m_hoodAngleMap.InsertValues(hoodMap);

    std::vector<std::pair<double, double>> flyMap{
        {1.34, 210.0},
        {1.78, 220.0},
        {2.17, 220.0},
        {2.81, 230.0},
        {3.82, 250.0},
        {4.09, 255.0},
        {4.40, 260.0},
        {4.77, 265.0},
        {5.57, 275.0},
        {5.60, 290.0}};
    m_flywheelMap.InsertValues(flyMap);

    std::vector<std::pair<double, double>> tofMap{
    {5.68, 1.16},
    {4.55, 1.12},
    {3.15, 1.11},
    {1.88, 1.09},
    {1.38, 0.90}};
    m_tofMap.InsertValues(tofMap);
}

frc::Translation2d ShotCalculator::FlipAlliance(frc::Translation2d t,
                                                  bool isRed) const {
  if (isRed) {
    return {units::meter_t{Constants::Field::kFieldLength} - t.X(), t.Y()};
  }
  return t;
}

ShotSolution ShotCalculator::Calculate(units::second_t now, bool isRed) {
  auto& rs = RobotState::Instance();

  units::second_t launchTime = now + m_cfg.totalLatency;
  OdometryData futureDrive = rs.GetDriveState(launchTime);

  frc::Pose2d launcherPose =
      futureDrive.pose.TransformBy(m_cfg.robotToLauncher);
  frc::Translation2d launcherXY = launcherPose.Translation();
  frc::Translation2d target = FlipAlliance(m_cfg.targetXY, isRed);

  double cos_h = futureDrive.pose.Rotation().Cos();
  double sin_h = futureDrive.pose.Rotation().Sin();
  double rvx = futureDrive.chassisSpeeds.vx.value();
  double rvy = futureDrive.chassisSpeeds.vy.value();

  double fieldVx = rvx * cos_h - rvy * sin_h;
  double fieldVy = rvx * sin_h + rvy * cos_h;

  double lx = launcherXY.X().value();
  double ly = launcherXY.Y().value();
  double tx = target.X().value();
  double ty = target.Y().value();

  double aimX = lx, aimY = ly;
  double effDist = std::hypot(tx - aimX, ty - aimY);
  double tof = m_tofMap.GetValue(effDist);

  for (int i = 0; i < m_cfg.maxAimIterations; i++) {
    double newAimX = lx + fieldVx * tof;
    double newAimY = ly + fieldVy * tof;
    double newDist = std::hypot(tx - newAimX, ty - newAimY);

    if (std::abs(newDist - effDist) < m_cfg.convergenceThresholdMeters) {
      aimX = newAimX;
      aimY = newAimY;
      effDist = newDist;
      break;
    }
    aimX = newAimX;
    aimY = newAimY;
    effDist = newDist;
    tof = m_tofMap.GetValue(effDist);
  }

  double hoodAngle = m_hoodAngleMap.GetValue(effDist);
  double flywheelSpeed = m_flywheelMap.GetValue(effDist);
  tof = m_tofMap.GetValue(effDist);

  frc::Rotation2d turretFieldAngle{
      units::radian_t{std::atan2(ty - aimY, tx - aimX)}};

  units::radian_t turretRobotAngle =
      (turretFieldAngle - futureDrive.pose.Rotation()).Radians();

  TurretState turret = rs.GetTurretState(now);

  FlywheelState flywheel = rs.GetFlywheelState(now);

  bool turretOK =
      units::math::abs(turret.angle - turretRobotAngle) <
      m_cfg.turretTolerance;
  bool hoodOK =
      std::abs(hoodAngle - 0.0 /* TODO: actual hood angle */) <
      m_cfg.hoodTolerance.value();
  bool flywheelOK =
      flywheel.velocity.value() > 0 &&
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
      .hoodAngle = units::radian_t{hoodAngle},
      .flywheelSpeed = flywheelSpeed,
      .effectiveDistance = units::meter_t{effDist},
      .timeOfFlight = units::second_t{tof},
  };
}
