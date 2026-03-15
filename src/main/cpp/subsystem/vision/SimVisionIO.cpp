// Team 5687 2026

#include "subsystem/vision/SimVisionIO.h"

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform3d.h>
#include <units/angle.h>
#include <units/length.h>

#include <algorithm>
#include <memory>
#include <vector>

#include "subsystem/vision/FieldConstants.h"
#include "subsystem/vision/Camera.h"
#include "subsystem/vision/SimulatedPhotonVisionCamera.h"
#include "subsystem/vision/VisionConstants.h"
#include "utils/vision/AprilTagObservation.h"

using units::degree_t;
using units::meter_t;

inline const AprilTagObservation *
BestTag(const std::vector<AprilTagObservation> &tags) {
  if (tags.empty())
    return nullptr;
  return &*std::ranges::max_element(
      tags, [](const AprilTagObservation &a, const AprilTagObservation &b) {
        if (a.Confidence() != b.Confidence())
          return a.Confidence() < b.Confidence();
        return a.Area() < b.Area();
      });
}

SimVisionIO::SimVisionIO()
    : m_visionSim(std::make_shared<photon::VisionSystemSim>("MainVision")) {
  m_visionSim->AddAprilTags(Constants::Field::kFieldTagLayout);

  m_cams.emplace("limelight",
                 std::make_unique<SimulatedPhotonVisionCamera>(
                     "limelight", Constants::Vision::kRobotToCam, m_visionSim));
}

void SimVisionIO::UpdateInputs(VisionIOInputs &inputs) {
  m_visionSim->Update(m_robotPose);

  inputs.cameraTagObservations.clear();
  inputs.visionPoseMeasurements.clear();

  for (auto &[name, cam] : m_cams) {
    Camera::VisionResult res = cam->GetLatestResult();

    inputs.cameraTagObservations.erase(name);
    if (!res.tags.empty()) {
      inputs.cameraTagObservations.emplace(name, std::move(res.tags));
    }

    if (res.poseEstimate) {
      inputs.visionPoseMeasurements.erase(name);
      inputs.visionPoseMeasurements.emplace(name, *res.poseEstimate);
    }
  }
}
