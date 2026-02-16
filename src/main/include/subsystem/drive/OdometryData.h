// Team 5687 2026

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <units/time.h>

#include <array>

#include "Constants.h"
#include "utils/Utils.h"

struct OdometryData {
  frc::Pose2d pose{};
  std::array<frc::SwerveModulePosition, Constants::SwerveDrive::kModuleCount>
      modulePositions{};
  std::array<frc::SwerveModuleState, Constants::SwerveDrive::kModuleCount>
      moduleStates{};
  frc::ChassisSpeeds chassisSpeeds{};
  frc::Rotation2d gyroAngle{};
  units::radians_per_second_t gyroRate{};
  units::second_t timestamp{0_s};
  bool isValid{false};

  OdometryData Extrapolate(units::second_t dt) const {
    OdometryData future = *this;
    future.timestamp = timestamp + dt;
    // Setting to 0 and adding rotation later looks more accurate
    // I think since it assumes I'm going along an arc the
    // ended estimate position is not correct with large dt.
    auto odometryDelta = frc::Twist2d{
        chassisSpeeds.vx * dt, chassisSpeeds.vy * dt, 0_rad,
        // chassisSpeeds.omega * dt,
    };

    future.pose = pose.Exp(odometryDelta);
    future.gyroAngle = gyroAngle + frc::Rotation2d(gyroRate * dt);
    future.pose = frc::Pose2d{{future.pose.Translation()}, {future.gyroAngle}};
    for (size_t i = 0; i < 4; i++) {
      future.modulePositions[i].distance = moduleStates[i].speed * dt;
      future.modulePositions[i].angle = moduleStates[i].angle;
    }

    return future;
  }

  static OdometryData Interpolate(const OdometryData& start,
                                  const OdometryData& end, double t) {
    OdometryData interpolated;

    interpolated.timestamp = Lerp(start.timestamp, end.timestamp, t);
    interpolated.pose = Lerp(start.pose, end.pose, t);
    interpolated.gyroAngle = Lerp(start.gyroAngle, end.gyroAngle, t);
    interpolated.gyroRate = Lerp(start.gyroRate, end.gyroRate, t);

    interpolated.chassisSpeeds.vx =
        Lerp(start.chassisSpeeds.vx, end.chassisSpeeds.vx, t);
    interpolated.chassisSpeeds.vy =
        Lerp(start.chassisSpeeds.vy, end.chassisSpeeds.vy, t);
    interpolated.chassisSpeeds.omega =
        Lerp(start.chassisSpeeds.omega, end.chassisSpeeds.omega, t);

    for (size_t i = 0; i < 4; i++) {
      interpolated.moduleStates[i].speed =
          Lerp(start.moduleStates[i].speed, end.moduleStates[i].speed, t);
      interpolated.moduleStates[i].angle =
          Lerp(start.moduleStates[i].angle, end.moduleStates[i].angle, t);

      interpolated.modulePositions[i].distance =
          Lerp(start.modulePositions[i].distance,
               end.modulePositions[i].distance, t);
      interpolated.modulePositions[i].angle =
          Lerp(start.modulePositions[i].angle, end.modulePositions[i].angle, t);
    }

    return interpolated;
  }
};
