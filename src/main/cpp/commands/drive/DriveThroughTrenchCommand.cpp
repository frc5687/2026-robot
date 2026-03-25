// Team 5687 2026

#include "commands/drive/DriveThroughTrenchCommand.h"

#include <frc/DriverStation.h>

#include "commands/drive/AutoAlignToPoseCommand.h"
#include "subsystem/vision/FieldConstants.h"

namespace DriveThroughTrenchCommand {

static void GetTrenchPoses(DriveSubsystem* drive, frc::Pose2d& outsidePose,
                           frc::Pose2d& insidePose) {
    using namespace Constants::Field;
    using namespace Constants::Field::Trench;

    frc::Translation2d pos = drive->GetPose().Translation();
    bool topHalf = pos.Y() > kFieldWidth / 2.0;

    bool isRed = false;
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance.has_value() &&
        alliance.value() == frc::DriverStation::Alliance::kRed) {
        isRed = true;
    }

    if (!isRed) {
        if (topHalf) {
            outsidePose = OutsideTopBlue;
            insidePose = InsideTopBlue;
        } else {
            outsidePose = OutsideBottomBlue;
            insidePose = InsideBottomBlue;
        }
    } else {
        if (topHalf) {
            outsidePose = OutsideTopRed;
            insidePose = InsideTopRed;
        } else {
            outsidePose = OutsideBottomRed;
            insidePose = InsideBottomRed;
        }
    }
}

frc2::CommandPtr Create(DriveSubsystem* driveSubsystem,
                        units::meter_t blendRadius, double constraintFactor) {
    auto passedEntrance = std::make_shared<bool>(false);

    frc::Pose2d outsidePose;
    frc::Pose2d insidePose;
    GetTrenchPoses(driveSubsystem, outsidePose, insidePose);

    frc::Translation2d pos = driveSubsystem->GetPose().Translation();
    double distToOutside = pos.Distance(outsidePose.Translation()).value();
    double distToInside = pos.Distance(insidePose.Translation()).value();

    frc::Pose2d firstPose = (distToOutside <= distToInside) ? outsidePose : insidePose;
    frc::Pose2d secondPose = (distToOutside <= distToInside) ? insidePose : outsidePose;

    return AutoAlignToPoseCommand(
               driveSubsystem,
               [driveSubsystem, blendRadius, passedEntrance, firstPose,
                secondPose]() -> frc::Pose2d {
                   if (!*passedEntrance) {
                       frc::Translation2d pos =
                           driveSubsystem->GetPose().Translation();
                       double dist =
                           pos.Distance(firstPose.Translation()).value();
                       if (dist < blendRadius.value()) {
                           *passedEntrance = true;
                       }
                   }
                   return *passedEntrance ? secondPose : firstPose;
               },
               constraintFactor)
        .ToPtr();
}

} // namespace DriveThroughTrenchCommand
