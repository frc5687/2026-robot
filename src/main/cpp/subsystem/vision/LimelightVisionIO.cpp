// Team 5687 2026

#include "subsystem/vision/LimelightVisionIO.h"

#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>
#include <units/length.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include "networktables/NetworkTableInstance.h"
#include "subsystem/vision/VisionIO.h"
#include "utils/Logger.h"
#include "utils/vision/AprilTagObservation.h"
#include "utils/vision/VisionMeasurement.h"

namespace {

// TODO move this was being lazy
constexpr double kMaxAmbiguity = 0.7;
constexpr double kLargeVariance = 9999.0;
constexpr double kFallbackXyStdDev = 0.1;
constexpr double kFallbackThetaStdDev = 0.1;

frc::Transform3d TransformFromArray(const std::vector<double> &arr) {
  if (arr.size() < 6)
    return frc::Transform3d{};
  constexpr double kD2R = std::numbers::pi / 180.0;
  return frc::Transform3d{frc::Translation3d{units::meter_t{arr[0]},
                                             units::meter_t{arr[1]},
                                             units::meter_t{arr[2]}},
                          frc::Rotation3d{units::radian_t{arr[3] * kD2R},
                                          units::radian_t{arr[4] * kD2R},
                                          units::radian_t{arr[5] * kD2R}}};
}

AprilTagObservation
ObservationFromFiducial(const LimelightHelpers::FiducialResultClass &fid,
                        units::second_t timestamp) {
  return AprilTagObservation{
      fid.m_fiducialID,
      TransformFromArray(fid.m_TargetTransform6DCAMERASPACE),
      /*ambiguity=*/0.0,
      fid.m_TargetAreaNormalized,
      units::degree_t{fid.m_TargetXDegreesCrosshairAdjusted},
      units::degree_t{fid.m_TargetYDegreesCrosshairAdjusted},
      timestamp,
      /*confidence=*/1.0};
}

const AprilTagObservation *
BestObservation(const std::vector<AprilTagObservation> &obs) {
  if (obs.empty())
    return nullptr;
  return &*std::ranges::max_element(
      obs, [](const AprilTagObservation &a, const AprilTagObservation &b) {
        if (a.Confidence() != b.Confidence())
          return a.Confidence() < b.Confidence();
        return a.Area() < b.Area();
      });
}

} // namespace

LimelightVisionIO::LimelightVisionIO(std::vector<std::string> limelightNames,
                                     MegaTagMode mode)
    : m_limelightNames{std::move(limelightNames)}, m_mode{mode} {}

void LimelightVisionIO::SetRobotOrientation(
    units::degree_t yaw, units::degrees_per_second_t yawRate,
    units::degree_t pitch, units::degrees_per_second_t pitchRate,
    units::degree_t roll, units::degrees_per_second_t rollRate) {
  for (const auto &name : m_limelightNames) {
    LimelightHelpers::SetRobotOrientation_NoFlush(
        name, yaw.value(), yawRate.value(), pitch.value(), pitchRate.value(),
        roll.value(), rollRate.value());
  }
  LimelightHelpers::Flush();
}

void LimelightVisionIO::UpdateInputs(VisionIOInputs &inputs) {
  inputs.cameraTagObservations.clear();
  inputs.visionPoseMeasurements.clear();
  for (const auto &name : m_limelightNames) {
    ProcessCamera(name, inputs);
  }
}

VisionMeasurement LimelightVisionIO::MeasurementFromEstimate(
    const std::string &limelightName, const LimelightHelpers::PoseEstimate &est,
    bool isMegaTag2) const {
  VisionMeasurement m;
  m.pose = est.pose;
  m.timestamp = est.timestampSeconds;
  m.tagCount = est.tagCount;
  m.avgTagDistance = est.avgTagDist;
  m.ambiguity = est.rawFiducials.empty() ? 0.0 : est.rawFiducials[0].ambiguity;

  if (isMegaTag2) {
    const double dist = std::max(est.avgTagDist, 0.01);
    const double countFactor = est.tagCount > 1 ? 0.5 : 1.0;
    m.xyStdDev = kFallbackXyStdDev * dist * dist * countFactor;
    m.thetaStdDev = kLargeVariance;
    //m.confidence = std::clamp(1.0 / (dist * dist), 0.0, 1.0);
    m.confidence = 1;
    return m;
  }

  // quality = 1.0 for multi-tag; (1 - ambiguity) for single-tag.
  const double quality =
      est.tagCount > 1 ? 1.0 : std::max(1.0 - m.ambiguity, 1e-6);

  const std::vector<double> fw =
      nt::NetworkTableInstance::GetDefault()
          .GetTable(LimelightHelpers::sanitizeName(limelightName))
          ->GetEntry("stddevs")
          .GetDoubleArray(std::vector<double>{});

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
  m.confidence = 1.0;
  return m;
}

void LimelightVisionIO::SetRobotPose(const frc::Pose2d &pose) {
  SetRobotOrientation(pose.Rotation().Degrees());
}

void LimelightVisionIO::ProcessCamera(const std::string &name,
                                      VisionIOInputs &inputs) {
  if (m_mode == MegaTagMode::kMegaTag1) {
    const auto est = LimelightHelpers::getBotPoseEstimate_wpiBlue(name);
    if (LimelightHelpers::validPoseEstimate(est)) {
      const bool highAmbiguity = est.tagCount == 1 &&
                                 !est.rawFiducials.empty() &&
                                 est.rawFiducials[0].ambiguity > kMaxAmbiguity;
      if (!highAmbiguity) {
        inputs.visionPoseMeasurements.emplace(
            name, MeasurementFromEstimate(name, est, /*isMegaTag2=*/false));
      }
    }
  } else {
    const auto est =
        LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(name);
    if (LimelightHelpers::validPoseEstimate(est)) {
      inputs.visionPoseMeasurements.emplace(
          name, MeasurementFromEstimate(name, est, /*isMegaTag2=*/true));
    }
  }

  // Per-tag observations from JSON, saw large loop times with json, so maybe
  // avoid
  const LimelightHelpers::LimelightResultsClass results =
      LimelightHelpers::getLatestResults(name);
  const auto &fiducials = results.targetingResults.FiducialResults;
  if (fiducials.empty())
    return;

  // Use the pose-estimate timestamp when available (already latency-corrected).
  units::second_t frameTimestamp;
  if (inputs.visionPoseMeasurements.count(name)) {
    frameTimestamp = inputs.visionPoseMeasurements.at(name).timestamp;
  } else {
    const double latSec = (results.targetingResults.m_latencyPipeline +
                           results.targetingResults.m_latencyCapture) /
                          1000.0;
    frameTimestamp =
        units::second_t{results.targetingResults.m_timeStamp - latSec};
  }

  std::vector<AprilTagObservation> observations;
  observations.reserve(fiducials.size());
  for (const auto &fid : fiducials) {
    observations.emplace_back(ObservationFromFiducial(fid, frameTimestamp));
  }
  // Logger::Instance().Log("Observation Count", observations.size());
  if (const AprilTagObservation *best = BestObservation(observations)) {
    inputs.cameraTagObservations.emplace(name, *best);
  }
}
