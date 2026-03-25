#pragma once

#include "frc2/command/CommandPtr.h"
#include "subsystem/drive/DriveSubsystem.h"
namespace DriveThroughTrenchCommand {
frc2::CommandPtr Create(DriveSubsystem* driveSubsystem, units::meter_t blendRadius = 1.0_m, double constraintFactor = 1.0);
}
