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

inline constexpr CANDevice HoodMotor{28, Bus::kShooter};

inline constexpr CANDevice RightFollowerFlywheel{20, Bus::kShooter};
inline constexpr CANDevice RightBottomFollowerFlywheel{21, Bus::kShooter};

inline constexpr CANDevice LeftLeaderFlywheel{22, Bus::kShooter};
inline constexpr CANDevice LeftFollowerFlywheel{23, Bus::kShooter};

inline constexpr CANDevice FeederLeader{24, Bus::kShooter};
inline constexpr CANDevice FeederFollower{25, Bus::kShooter};

inline constexpr CANDevice KickerLeader{27, Bus::kShooter};   // left
inline constexpr CANDevice KickerFollower{26, Bus::kShooter}; // right

inline constexpr CANDevice IntakeDeployer{19, Bus::kDriveTrain};

inline constexpr CANDevice IntakeTopRollerLeader{30, Bus::kRio};
inline constexpr CANDevice IntakeTopRollerFollower{31, Bus::kRio};

inline constexpr CANDevice IntakeBottomRoller{13, Bus::kDriveTrain};

} // namespace TalonFX

namespace CANCoder {
inline constexpr CANDevice FrontLeftEncoder{2, Bus::kDriveTrain};
inline constexpr CANDevice FrontRightEncoder{1, Bus::kDriveTrain};
inline constexpr CANDevice BackLeftEncoder{3, Bus::kDriveTrain};
inline constexpr CANDevice BackRightEncoder{0, Bus::kDriveTrain};

inline constexpr CANDevice HoodEncoder{0, Bus::kShooter};
} // namespace CANCoder

namespace Pidgeon2 {
inline constexpr CANDevice IMU{0, Bus::kDriveTrain};
} // namespace Pidgeon2

} // namespace CAN

} // namespace HardwareMap
