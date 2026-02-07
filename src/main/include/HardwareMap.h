
#pragma once

#include <string_view>

#include "utils/CANDevice.h"

namespace HardwareMap {
namespace Bus {
inline constexpr std::string_view kDriveTrain = "DriveTrain";
inline constexpr std::string_view kRio = "rio";
inline constexpr std::string_view kStructure = "SuperStructure";
} // namespace Bus

namespace CAN {
namespace TalonFX {
inline constexpr CANDevice FrontLeftDrive{5, Bus::kDriveTrain};
inline constexpr CANDevice FrontLeftSteer{4, Bus::kDriveTrain};

inline constexpr CANDevice FrontRightDrive{3, Bus::kDriveTrain};
inline constexpr CANDevice FrontRightSteer{2, Bus::kDriveTrain};

inline constexpr CANDevice BackLeftDrive{7, Bus::kDriveTrain};
inline constexpr CANDevice BackLeftSteer{6, Bus::kDriveTrain};

inline constexpr CANDevice BackRightDrive{1, Bus::kDriveTrain};
inline constexpr CANDevice BackRightSteer{0, Bus::kDriveTrain};

inline constexpr CANDevice LeftRollerMotor{8, Bus::kRio};
inline constexpr CANDevice RightRollerMotor{9, Bus::kRio};

inline constexpr CANDevice LeftIndexerMotor{10, Bus::kDriveTrain};
inline constexpr CANDevice CenterIndexerMotor{11, Bus::kDriveTrain};
inline constexpr CANDevice RightIndexerMotor{12, Bus::kDriveTrain};


inline constexpr CANDevice LinearIntake{13, Bus::kDriveTrain};
inline constexpr CANDevice LeftElevator{9, Bus::kDriveTrain};
inline constexpr CANDevice RightElevator{8, Bus::kDriveTrain};

inline constexpr CANDevice LeftFlywheel{14, Bus::kDriveTrain};
inline constexpr CANDevice RightFlywheel{15, Bus::kDriveTrain};

} // namespace TalonFX

namespace CANCoder {
inline constexpr CANDevice FrontLeftEncoder{2, Bus::kDriveTrain};
inline constexpr CANDevice FrontRightEncoder{1, Bus::kDriveTrain};
inline constexpr CANDevice BackLeftEncoder{3, Bus::kDriveTrain};
inline constexpr CANDevice BackRightEncoder{0, Bus::kDriveTrain};
inline constexpr CANDevice HoodEncoder{5, Bus::kDriveTrain};
} // namespace CANCoder

namespace Pidgeon2 {
inline constexpr CANDevice IMU{0, Bus::kDriveTrain};
} // namespace Pidgeon2

} // namespace CAN

} // namespace HardwareMap
