// Team 5687 2026

#pragma once

#include <string_view>

#include "utils/CANDevice.h"

namespace HardwareMap {
namespace Bus {
inline constexpr std::string_view kDriveTrain = "DriveTrain";
inline constexpr std::string_view kRio = "rio";
inline constexpr std::string_view kShooter = "Shooter";
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

inline constexpr CANDevice Turret{20, Bus::kDriveTrain};

inline constexpr CANDevice RightLeaderFlywheel{16, Bus::kShooter};
inline constexpr CANDevice RightFollowerFlywheel{17, Bus::kShooter};

inline constexpr CANDevice LeftLeaderFlywheel{14, Bus::kShooter};
inline constexpr CANDevice LeftFollowerFlywheel{15, Bus::kShooter};

inline constexpr CANDevice LeftIndexer{10, Bus::kDriveTrain};
inline constexpr CANDevice CenterIndexer{11, Bus::kDriveTrain};
inline constexpr CANDevice RightIndexer{12, Bus::kDriveTrain};



inline constexpr CANDevice LeftRollerMotor{8, Bus::kRio};
inline constexpr CANDevice RightRollerMotor{9, Bus::kRio};

} // namespace TalonFX

namespace CANCoder {
inline constexpr CANDevice FrontLeftEncoder{2, Bus::kDriveTrain};
inline constexpr CANDevice FrontRightEncoder{1, Bus::kDriveTrain};
inline constexpr CANDevice BackLeftEncoder{3, Bus::kDriveTrain};
inline constexpr CANDevice BackRightEncoder{0, Bus::kDriveTrain};
} // namespace CANCoder

namespace Pidgeon2 {
inline constexpr CANDevice IMU{0, Bus::kDriveTrain};
} // namespace Pidgeon2

} // namespace CAN

} // namespace HardwareMap
