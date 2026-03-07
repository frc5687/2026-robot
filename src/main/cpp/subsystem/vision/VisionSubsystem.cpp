// Team 5687 2026

#include "subsystem/vision/VisionSubsystem.h"

#include <frc/Timer.h>

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "subsystem/vision/VisionConstants.h"

VisionSubsystem::VisionSubsystem(std::unique_ptr<VisionIO> io,
                                 std::shared_ptr<OdometryThread> odometryThread)
    : LoggedSubsystem("Vision"), m_io(std::move(io)),
      m_odometryThread(odometryThread) {}

void VisionSubsystem::UpdateInputs() {
  // This is really only for Sim
  SetRobotPose(m_odometryThread->GetOdometryPose());

  m_io->UpdateInputs(m_inputs);

  m_totalUpdates++;
  if (!m_inputs.visionPoseMeasurements.empty()) {
    m_updatesWithMeasurements++;
  }

  for (const auto &measurement : m_inputs.visionPoseMeasurements) {
    m_odometryThread->AddVisionMeasurement(measurement.second);
  }
}

void VisionSubsystem::LogTelemetry() {
  for (const auto &est : m_inputs.visionPoseMeasurements) {
    std::string key = "Estimated Pose" + est.first;
    Log(key, est.second.pose);
  }

  std::vector<frc::Pose3d> tags;
  for (const auto &[cam, observations] : m_inputs.cameraTagObservations) {
    auto it = Constants::Vision::kTransformMap.find(cam);
    if (it == Constants::Vision::kTransformMap.end()) {
      std::cerr << "Failed to find camera: " << cam << "\n";
      continue;
    }
    frc::Transform3d cameraTransform = it->second;
    frc::Pose3d robotPose3d{m_odometryThread->GetEstimatedPose()};

    for (const auto &obs : observations) {
      frc::Pose3d tagPoseInField =
          robotPose3d.TransformBy(cameraTransform).TransformBy(obs.Transform());
      tags.push_back(tagPoseInField);
    }
  }

  Log("Tag Poses", tags);
  constexpr std::array<frc::Transform3d, 1> camTransforms{
      Constants::Vision::kRobotToCam,
  };

  Log("Transforms", camTransforms);
  Log("TotalUpdates", m_totalUpdates);
  Log("UpdatesWithMeasurements", m_updatesWithMeasurements);
  size_t tagsSeen = 0;
  for (const auto &[name, observations] : m_inputs.cameraTagObservations) {
    tagsSeen += observations.size();
  }
  Log("TagsSeenThisLoop", tagsSeen);
  Log("MeasurementsThisLoop", m_inputs.visionPoseMeasurements.size());

  auto now = frc::Timer::GetFPGATimestamp();
  for (const auto &[name, m] : m_inputs.visionPoseMeasurements) {
    std::string prefix = name + "/";
    Log(prefix + "TagCount", m.tagCount);
    Log(prefix + "AvgTagDist", m.avgTagDistance);
    Log(prefix + "Confidence", m.confidence);
    Log(prefix + "Ambiguity", m.ambiguity);
    Log(prefix + "XYStdDev", m.xyStdDev);
    Log(prefix + "ThetaStdDev", m.thetaStdDev);
    Log(prefix + "Latency", (now - m.timestamp).value());
    Log(prefix + "Pose", m.pose);
  }

  int totalTags = 0;
  for (const auto &[name, observations] : m_inputs.cameraTagObservations) {
    for (const auto &obs : observations) {
      std::string prefix = name + "/Tag" + std::to_string(totalTags) + "/";
      Log(prefix + "ID", obs.Id());
      Log(prefix + "Ambiguity", obs.Ambiguity());
      Log(prefix + "Area", obs.Area());
      Log(prefix + "Confidence", obs.Confidence());
      totalTags++;
    }
  }
  Log("TagCount", totalTags);
}
