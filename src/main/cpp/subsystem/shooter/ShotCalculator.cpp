// Team 5687 2026

#include "subsystem/shooter/ShotCalculator.h"

#include <cmath>
#include <numbers>

#include "Constants.h"
#include "RobotState.h"
#include "utils/Logger.h"

ShotCalculator::ShotCalculator() {
  // distance (m) → hood angle (turns)
  m_hoodAngleMap.InsertValues({
      {1.524, 0.15}, {2.1336, 0.27}, {2.7432, 0.35}, {3.6576, 0.42},
      {4.2672, 0.44}, {4.8768, 0.46}, {5.1816, 0.47},
  });

  // distance (m) → flywheel speed (units TBD)
  m_flywheelMap.InsertValues({
      {1.524, 1675}, {2.1336, 1800}, {2.7432, 1900}, {3.6576, 2160},
      {4.2672, 2350}, {4.8768, 2475}, {5.1816, 2500},
  });

  // distance (m) → time of flight (s)
  m_tofMap.InsertValues({
      {1.524, 0.9}, {2.1336, 0.9}, {2.7432, 1.0}, {3.6576, 1.05},
      {4.2672, 1.13}, {4.8768, 1.3}, {5.1816, 1.3},
  });
}

units::radian_t ShotCalculator::Normalize0To2Pi(units::turn_t angle) {
  units::radian_t radian = units::radian_t{angle};
  double a = std::fmod(radian.value(), 2.0 * std::numbers::pi);
  if (a < 0.0) {
    a += 2.0 * std::numbers::pi;
  }
  return units::radian_t{a};
}

units::radian_t ShotCalculator::AngleDifference(frc::Rotation2d a,
                                                frc::Rotation2d b) {
  return (a - b).Radians();
}

frc::Rotation2d ShotCalculator::AngularBisector(frc::Rotation2d a,
                                                frc::Rotation2d b) {
  double cx = a.Cos() + b.Cos();
  double cy = a.Sin() + b.Sin();
  return frc::Rotation2d{units::radian_t{std::atan2(cy, cx)}};
}

frc::Translation2d ShotCalculator::FlipAlliance(frc::Translation2d t,
                                                bool isRed) const {
  if (isRed) {
    return {units::meter_t{Constants::Field::kFieldLength} - t.X(), t.Y()};
  }
  return t;
}

LauncherSolution ShotCalculator::SolveSingleLauncher(
    frc::Translation2d launcherXY,
    double fieldVx, double fieldVy,
    frc::Translation2d target) const {

  double lx = launcherXY.X().value();
  double ly = launcherXY.Y().value();
  double tx = target.X().value();
  double ty = target.Y().value();

  // Iterative lookahead compensating for robot motion during TOF.
  double effDist = std::hypot(tx - lx, ty - ly);
  double tof     = m_tofMap.GetValue(effDist);

  double lookaheadX = lx;
  double lookaheadY = ly;

  for (int i = 0; i < m_cfg.maxAimIterations; ++i) {
    double newX    = lx + fieldVx * tof;
    double newY    = ly + fieldVy * tof;
    double newDist = std::hypot(tx - newX, ty - newY);

    lookaheadX = newX;
    lookaheadY = newY;

    if (std::abs(newDist - effDist) < m_cfg.convergenceThresholdMeters) {
      effDist = newDist;
      break;
    }

    effDist = newDist;
    tof     = m_tofMap.GetValue(effDist);
  }

  frc::Rotation2d idealAngle{
      units::radian_t{std::atan2(ty - lookaheadY, tx - lookaheadX)}};

  // Each launcher gets its own hood/flywheel/TOF from its own distance.
  return {
      .fieldPosition    = launcherXY,
      .effectiveDistance = effDist,
      .idealFieldAngle  = idealAngle,
      .hoodAngle        = units::turn_t{m_hoodAngleMap.GetValue(effDist)},
      .flywheelSpeed    = m_flywheelMap.GetValue(effDist),
      .timeOfFlight     = units::second_t{m_tofMap.GetValue(effDist)},
  };
}

ShotSolution ShotCalculator::Calculate(units::second_t now, bool isRed) {
  auto& rs = RobotState::Instance();

  units::second_t launchTime = now + m_cfg.totalLatency;
  OdometryData futureDrive   = rs.GetDriveState(launchTime);

  frc::Pose2d robotPose   = futureDrive.estimatedPose;
  frc::Rotation2d heading  = robotPose.Rotation();
  double cos_h = heading.Cos();
  double sin_h = heading.Sin();

  frc::Pose2d leftPose  = robotPose.TransformBy(m_cfg.robotToLeftLauncher);
  frc::Pose2d rightPose = robotPose.TransformBy(m_cfg.robotToRightLauncher);

  Logger::Instance().Log("ShotCalculator/leftPose",  leftPose);
  Logger::Instance().Log("ShotCalculator/rightPose", rightPose);

  double rvx   = futureDrive.chassisSpeeds.vx.value();
  double rvy   = futureDrive.chassisSpeeds.vy.value();
  double omega = futureDrive.chassisSpeeds.omega.value();

  double baseFieldVx = rvx * cos_h - rvy * sin_h;
  double baseFieldVy = rvx * sin_h + rvy * cos_h;

  auto launcherFieldVelocity = [&](const frc::Transform2d& offset)
      -> std::pair<double, double> {
    double ox = offset.X().value();
    double oy = offset.Y().value();
    double oxField = ox * cos_h - oy * sin_h;
    double oyField = ox * sin_h + oy * cos_h;
    return {
        baseFieldVx + (-omega * oyField),
        baseFieldVy + ( omega * oxField),
    };
  };

  auto [leftVx,  leftVy]  = launcherFieldVelocity(m_cfg.robotToLeftLauncher);
  auto [rightVx, rightVy] = launcherFieldVelocity(m_cfg.robotToRightLauncher);

  frc::Translation2d target = FlipAlliance(m_cfg.targetXY, isRed);

  LauncherSolution left  = SolveSingleLauncher(
      leftPose.Translation(),  leftVx,  leftVy,  target);
  LauncherSolution right = SolveSingleLauncher(
      rightPose.Translation(), rightVx, rightVy, target);


  frc::Rotation2d turretFieldAngle =
      AngularBisector(left.idealFieldAngle, right.idealFieldAngle);

  units::radian_t divergence =
      units::math::abs(AngleDifference(left.idealFieldAngle,
                                       right.idealFieldAngle));

  units::radian_t turretRobotAngle =
      Normalize0To2Pi((turretFieldAngle - heading).Radians());

  turretRobotAngle = std::clamp(
      turretRobotAngle,
      Constants::Turret::kMinAngle,
      Constants::Turret::kMaxAngle);

  TurretState   turret   = rs.GetTurretState(now);
  HoodState     hood     = rs.GetHoodState(now);      // TODO: left/right hoods
  FlywheelState flywheel = rs.GetFlywheelState(now);   // TODO: left/right flywheels

  bool turretOK =
      units::math::abs(turret.angle - turretRobotAngle) <
      m_cfg.turretTolerance;

  bool leftHoodOK =
      units::math::abs(hood.angle - left.hoodAngle) <
      m_cfg.hoodTolerance;

  bool rightHoodOK =
      units::math::abs(hood.angle - right.hoodAngle) <
      m_cfg.hoodTolerance;

  bool leftFlywheelOK =
      left.flywheelSpeed > 0.0 &&
      std::abs(flywheel.velocity.value() - left.flywheelSpeed) /
              std::max(left.flywheelSpeed, 0.1) <
          m_cfg.flywheelToleranceFrac;

  bool rightFlywheelOK =
      right.flywheelSpeed > 0.0 &&
      std::abs(flywheel.velocity.value() - right.flywheelSpeed) /
              std::max(right.flywheelSpeed, 0.1) <
          m_cfg.flywheelToleranceFrac;

  bool inRange =
      left.effectiveDistance  >= m_cfg.minDistanceMeters &&
      left.effectiveDistance  <= m_cfg.maxDistanceMeters &&
      right.effectiveDistance >= m_cfg.minDistanceMeters &&
      right.effectiveDistance <= m_cfg.maxDistanceMeters;

  bool divergenceOK = divergence < m_cfg.maxDivergence;

  return {
      .inRange            = inRange,
      .ready              = inRange && divergenceOK && turretOK &&
                            leftHoodOK && rightHoodOK &&
                            leftFlywheelOK && rightFlywheelOK,
      .divergenceOK       = divergenceOK,
      .turretFieldAngle   = turretFieldAngle,
      .turretRobotAngle   = turretRobotAngle,
      .leftHoodAngle      = left.hoodAngle,
      .leftFlywheelSpeed  = left.flywheelSpeed,
      .leftDistance        = units::meter_t{left.effectiveDistance},
      .rightHoodAngle     = right.hoodAngle,
      .rightFlywheelSpeed = right.flywheelSpeed,
      .rightDistance       = units::meter_t{right.effectiveDistance},
      .angularDivergence  = divergence,
  };
}