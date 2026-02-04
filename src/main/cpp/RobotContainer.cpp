// RobotContainer.cpp
#include "RobotContainer.h"

#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/length.h>

#include <array>
#include <memory>
#include <utility>

#include "HardwareMap.h"
#include "commands/drive/DriveMaintainingHeadingCommand.h"
#include "subsystem/drive/PigeonIO.h"
#include "subsystem/drive/SimGyroIO.h"
#include "subsystem/drive/module/ModuleConfig.h"
#include "subsystem/drive/module/SimModuleIO.h"
#include "commands/drive/DriveWithNormalVectorAlignment.h"
#include "subsystem/drive/module/CTREModuleIO.h"
#include "subsystem/intake/IntakeSubsystem.h"
#include "subsystem/intake/linearintake/CTRELinearIntakeIO.h"
#include "subsystem/intake/linearintake/SimLinearIntakeIO.h"
#include "subsystem/vision/SimVisionIO.h"
#include "subsystem/intake/linearintake/CTRELinearIntakeIO.h"
#include "subsystem/intake/linearintake/SimLinearIntakeIO.h"

RobotContainer::RobotContainer() {
  m_drive = CreateDrive();
//   m_intakeRoller = CreateIntakeRoller();
  m_intakeSubsystem = CreateIntakeSubsystem();
 // m_linearIntake = CreateLinearIntake();
//   m_elevator = CreateElevator();
  //m_vision = CreateVision();
  ConfigureBindings();
}

std::unique_ptr<DriveSubsystem> RobotContainer::CreateDrive() {
  // Module encoder offsets (tune these per robot)
  constexpr std::array<units::turn_t, 4> kEncoderOffsets{
      0.3505859375_tr,               // FL
      -0.05517578125_tr,    // FR
      0.27099609375_tr - 0.5_tr, // BL
      0.096923828125_tr  // BR
      };
    
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<DriveSubsystem>(
        std::make_unique<SimModuleIO>(
            ModuleConfig{ModulePosition::FrontLeft, 0_tr}),
        std::make_unique<SimModuleIO>(
            ModuleConfig{ModulePosition::FrontRight, 0_tr}),
        std::make_unique<SimModuleIO>(
            ModuleConfig{ModulePosition::BackLeft, 0_tr}),
        std::make_unique<SimModuleIO>(
            ModuleConfig{ModulePosition::BackRight, 0_tr}),
        std::make_unique<SimGyroIO>());
  }

  // Real hardware
  return std::make_unique<DriveSubsystem>(
      std::make_unique<CTREModuleIO>(
          CTREModuleIO::DeviceIDs{
              HardwareMap::CAN::TalonFX::FrontLeftDrive,
              HardwareMap::CAN::TalonFX::FrontLeftSteer,
              HardwareMap::CAN::CANCoder::FrontLeftEncoder},
          ModuleConfig{ModulePosition::FrontLeft, kEncoderOffsets[0]}),
      
      std::make_unique<CTREModuleIO>(
          CTREModuleIO::DeviceIDs{
              HardwareMap::CAN::TalonFX::FrontRightDrive,
              HardwareMap::CAN::TalonFX::FrontRightSteer,
              HardwareMap::CAN::CANCoder::FrontRightEncoder},
          ModuleConfig{ModulePosition::FrontRight, kEncoderOffsets[1]}),
      
      std::make_unique<CTREModuleIO>(
          CTREModuleIO::DeviceIDs{
              HardwareMap::CAN::TalonFX::BackLeftDrive,
              HardwareMap::CAN::TalonFX::BackLeftSteer,
              HardwareMap::CAN::CANCoder::BackLeftEncoder},
          ModuleConfig{ModulePosition::BackLeft, kEncoderOffsets[2]}),
      
      std::make_unique<CTREModuleIO>(
          CTREModuleIO::DeviceIDs{
              HardwareMap::CAN::TalonFX::BackRightDrive,
              HardwareMap::CAN::TalonFX::BackRightSteer,
              HardwareMap::CAN::CANCoder::BackRightEncoder},
          ModuleConfig{ModulePosition::BackRight, kEncoderOffsets[3]}),
      
      std::make_unique<PigeonIO>(HardwareMap::CAN::Pidgeon2::IMU));
}



std::unique_ptr<IntakeRoller> RobotContainer::CreateIntakeRoller(){
    return std::make_unique<IntakeRoller>(
        std::make_unique<CTREIntakeRollerIO>(
            HardwareMap::CAN::TalonFX::LeftRollerMotor,
            HardwareMap::CAN::TalonFX::RightRollerMotor
        ));
}

std::unique_ptr<IntakeSubsystem> RobotContainer::CreateIntakeSubsystem(){
    return std::make_unique<IntakeSubsystem>(
        std::make_unique<CTRELinearIntakeIO>(
            HardwareMap::CAN::TalonFX::LinearIntake
        ),
        std::make_unique<CTREIntakeRollerIO>(
            HardwareMap::CAN::TalonFX::LeftRollerMotor,
            HardwareMap::CAN::TalonFX::RightRollerMotor
        )
    );
}

// std::unique_ptr<LinearIntake> RobotContainer::CreateLinearIntake(){
//     if(frc::RobotBase::IsSimulation()){
//         return std::make_unique<LinearIntake>(
//             std::make_unique<SimLinearIntakeIO>());
//     }

//     return std::make_unique<LinearIntake>(
//         std::make_unique<CTRELinearIntakeIO>(
//             HardwareMap::CAN::TalonFX::LinearIntake
//         ));
// }
// std::unique_ptr<ElevatorSubsystem> RobotContainer::CreateElevator() {
//   if (frc::RobotBase::IsSimulation()) {
//     return std::make_unique<ElevatorSubsystem>(
//         std::make_unique<SimElevatorIO>());
//   }

//   return std::make_unique<ElevatorSubsystem>(
//       std::make_unique<CTREElevatorIO>(
//           HardwareMap::CAN::TalonFX::LeftElevator,
//           HardwareMap::CAN::TalonFX::RightElevator));
// }

// std::unique_ptr<VisionSubsystem> RobotContainer::CreateVision() {
//   return std::make_unique<VisionSubsystem>(
//       std::make_unique<SimVisionIO>(),
//       m_drive->GetOdometryThread());
// }

void RobotContainer::ConfigureBindings() {
  using frc2::cmd::Run;

  //Set default drive command
  m_drive->SetDefaultCommand(DriveMaintainingHeadingCommand(
      m_drive.get(),
      [this] { return -m_driver.GetLeftY(); },
      [this] { return -m_driver.GetLeftX(); },
      [this] { return -m_driver.GetRightX(); },
      false)); //s lew limiter

//   m_driver.Square().WhileTrue(
//       DriveWithNormalVectorAlignment(
//           m_drive.get(),
//           []() { return frc::Pose2d{5_m, 3_m, frc::Rotation2d{45_deg}}; },
//           false)
//       .ToPtr());

    m_driver.R1().OnTrue(Run(
      [this] { m_intakeSubsystem->SetVoltage(0.0_V); }, {m_intakeRoller.get()}));

    m_driver.L1().OnTrue(Run(
      [this] { m_intakeSubsystem->SetVoltage(9_V); }, {m_intakeRoller.get()}));

    m_driver.Triangle().OnTrue(Run(
      [this] { m_intakeSubsystem->SetVoltage(-9_V); }, {m_intakeRoller.get()})); 

    // m_driver.Circle().OnTrue(Run(
    //   [this] { m_linearIntake->SetPosition(1.0_m); }, {m_linearIntake.get()}));

    // m_driver.Cross().OnTrue(Run(
    //   [this] { m_linearIntake->SetPosition(0.0_m); }, {m_linearIntake.get()}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}