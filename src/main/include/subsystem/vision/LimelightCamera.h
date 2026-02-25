// Team 5687 2026

#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include "subsystem/vision/Camera.h"
#include "utils/vision/AprilTagObservation.h"
#include "utils/vision/LimelightHelpers.h"
#include "utils/vision/VisionMeasurement.h"

// A single Limelight camera implementing the Camera interface, mirroring
// PhotonVisionCamera. All data comes from typed NT entries — no JSON parsing.
// Tags are derived from PoseEstimate.rawFiducials (embedded in the same typed
// NT read as the pose estimate), so GetLatestResult() costs exactly one NT
// double-array read per call.
class LimelightCamera : public Camera {
public:
  enum class MegaTagMode { kMegaTag1, kMegaTag2 };

  struct Config {
    std::string name;
    MegaTagMode mode{MegaTagMode::kMegaTag1};
    double maxAmbiguity{0.7};
  };

  explicit LimelightCamera(Config cfg) : m_cfg{std::move(cfg)} {}

  const std::string &Name() const { return m_cfg.name; }

  // Call once per camera per loop before Flush(). Needed for MegaTag2.
  void SetRobotOrientationNoFlush(
      units::degree_t yaw,
      units::degrees_per_second_t yawRate = 0.0_deg_per_s,
      units::degree_t pitch = 0.0_deg,
      units::degrees_per_second_t pitchRate = 0.0_deg_per_s,
      units::degree_t roll = 0.0_deg,
      units::degrees_per_second_t rollRate = 0.0_deg_per_s) {
    LimelightHelpers::SetRobotOrientation_NoFlush(
        m_cfg.name, yaw.value(), yawRate.value(), pitch.value(),
        pitchRate.value(), roll.value(), rollRate.value());
  }

  // Returns tag observations and optional pose estimate using only cached
  // typed NT reads. No JSON parsing.
  VisionResult GetLatestResult() override {
    LimelightHelpers::PoseEstimate est =
        (m_cfg.mode == MegaTagMode::kMegaTag2)
            ? LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(m_cfg.name)
            : LimelightHelpers::getBotPoseEstimate_wpiBlue(m_cfg.name);

    VisionResult out{};

    if (!est.rawFiducials.empty()) {
      out.tags.reserve(est.rawFiducials.size());

      // Primary target's 6DOF camera-space transform: one typed NT array read,
      // no JSON. Only meaningful for index 0 (the primary target).
      const frc::Transform3d primaryTransform = TransformFromArray(
          LimelightHelpers::getTargetPose_CameraSpace(m_cfg.name));

      for (size_t i = 0; i < est.rawFiducials.size(); ++i) {
        const LimelightHelpers::RawFiducial &fid = est.rawFiducials[i];
        out.tags.emplace_back(
            fid.id, (i == 0) ? primaryTransform : frc::Transform3d{},
            fid.ambiguity, fid.ta, units::degree_t{fid.txnc},
            units::degree_t{fid.tync}, est.timestampSeconds,
            std::clamp(1.0 - fid.ambiguity, 0.0, 1.0));
      }
    }

    if (LimelightHelpers::validPoseEstimate(est)) {
      const bool highAmbiguity = m_cfg.mode == MegaTagMode::kMegaTag1 &&
                                 est.tagCount == 1 &&
                                 !est.rawFiducials.empty() &&
                                 est.rawFiducials[0].ambiguity > m_cfg.maxAmbiguity;
      if (!highAmbiguity) {
        out.poseEstimate = BuildMeasurement(est);
      }
    }

    return out;
  }

private:
  static constexpr double kLargeVariance = 9999.0;
  static constexpr double kFallbackXyStdDev = 0.1;
  static constexpr double kFallbackThetaStdDev = 0.1;

  static frc::Transform3d TransformFromArray(const std::vector<double> &arr) {
    if (arr.size() < 6)
      return frc::Transform3d{};
    constexpr double kD2R = std::numbers::pi / 180.0;
    return frc::Transform3d{
        frc::Translation3d{units::meter_t{arr[0]}, units::meter_t{arr[1]},
                           units::meter_t{arr[2]}},
        frc::Rotation3d{units::radian_t{arr[3] * kD2R},
                        units::radian_t{arr[4] * kD2R},
                        units::radian_t{arr[5] * kD2R}}};
  }

  VisionMeasurement
  BuildMeasurement(const LimelightHelpers::PoseEstimate &est) const {
    VisionMeasurement m;
    m.pose = est.pose;
    m.pose3d = frc::Pose3d{est.pose};
    m.timestamp = est.timestampSeconds;
    m.tagCount = est.tagCount;
    m.avgTagDistance = est.avgTagDist;
    m.ambiguity =
        est.rawFiducials.empty() ? 0.0 : est.rawFiducials[0].ambiguity;

    if (m_cfg.mode == MegaTagMode::kMegaTag2) {
      const double dist = std::max(est.avgTagDist, 0.01);
      const double countFactor = est.tagCount > 1 ? 0.5 : 1.0;
      m.xyStdDev = kFallbackXyStdDev * dist * dist * countFactor;
      m.thetaStdDev = kLargeVariance;
      m.confidence = std::clamp(1.0 / (dist * dist), 0.0, 1.0);
      return m;
    }

    // MegaTag1: quality-weighted stddevs from Limelight firmware or fallback.
    const double quality =
        est.tagCount > 1 ? 1.0 : std::max(1.0 - m.ambiguity, 1e-6);
    const std::vector<double> fw =
        LimelightHelpers::getLimelightNTDoubleArray(m_cfg.name, "stddevs");

    if (fw.size() >= 3 && (fw[0] > 0.0 || fw[1] > 0.0)) {
      m.xyStdDev = std::max(fw[0], fw[1]) / quality;
      m.thetaStdDev = fw[2] / quality;
    } else {
      const double dist = std::max(est.avgTagDist, 0.01);
      const double countFactor = est.tagCount > 1 ? 0.5 : 1.0;
      m.xyStdDev = (kFallbackXyStdDev * dist * dist * countFactor) / quality;
      m.thetaStdDev = (kFallbackThetaStdDev * dist * countFactor) / quality;
    }

    m.confidence = std::clamp(
        quality / std::max(est.avgTagDist * est.avgTagDist, 0.01), 0.0, 1.0);
    return m;
  }

  Config m_cfg;
};
