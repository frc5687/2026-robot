// Team 5687 2026

#include "subsystem/vision/LimelightVisionIO.h"

#include <algorithm>
#include <utility>

#include "utils/vision/LimelightHelpers.h"

LimelightVisionIO::LimelightVisionIO(
    std::vector<LimelightCamera::Config> cameras) {
  m_cameras.reserve(cameras.size());
  for (auto &cfg : cameras) {
    m_cameras.emplace_back(std::move(cfg));
  }
}

void LimelightVisionIO::SetRobotPose(const frc::Pose2d &pose) {
  for (auto &cam : m_cameras) {
    cam.SetRobotOrientationNoFlush(pose.Rotation().Degrees());
  }
  LimelightHelpers::Flush();
}

void LimelightVisionIO::UpdateInputs(VisionIOInputs &inputs) {
  inputs.cameraTagObservations.clear();
  inputs.visionPoseMeasurements.clear();

  for (auto &cam : m_cameras) {
    Camera::VisionResult result = cam.GetLatestResult();

    if (result.poseEstimate) {
      inputs.visionPoseMeasurements.emplace(cam.Name(),
                                            std::move(*result.poseEstimate));
    }

    if (!result.tags.empty()) {
      const AprilTagObservation &best = *std::ranges::max_element(
          result.tags,
          [](const AprilTagObservation &a, const AprilTagObservation &b) {
            if (a.Confidence() != b.Confidence())
              return a.Confidence() < b.Confidence();
            return a.Area() < b.Area();
          });
      inputs.cameraTagObservations.emplace(cam.Name(), best);
    }
  }
}
