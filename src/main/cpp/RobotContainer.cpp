// Team 5687 2026

// RobotContainer.cpp
#include "RobotContainer.h"

#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/power.h>
#include <units/voltage.h>

#include <memory>

#include "HardwareMap.h"
#include "RobotState.h"
#include "commands/drive/DriveMaintainingHeadingCommand.h"
#include "commands/drive/AlignToEdgeCommand.h"
#include "commands/intake/EjectIntakeCommand.h"
#include "commands/intake/IntakeCommand.h"
#include "commands/intake/StopIntakeCommand.h"
#include "commands/shooter/AutoShootCommand.h"
#include "commands/shooter/ShootCommand.h"
#include "commands/shooter/ShotCalculatedSpinUpCommand.h"
#include "commands/shooter/SimpleShootCommand.h"
#include "frc/Timer.h"
#include "pathplanner/lib/auto/AutoBuilder.h"
#include "subsystem/drive/PigeonIO.h"
#include "subsystem/drive/SimGyroIO.h"
#include "subsystem/drive/module/CTREModuleIO.h"
#include "subsystem/drive/module/ModuleConfig.h"
#include "subsystem/drive/module/SimModuleIO.h"
#include "subsystem/feeder/CTREFeederIO.h"
#include "subsystem/feeder/SimFeederIO.h"
#include "subsystem/flywheel/CTREFlywheelIO.h"
#include "subsystem/flywheel/SimFlywheelIO.h"
#include "subsystem/hood/CTREHoodIO.h"
#include "subsystem/hood/SimHoodIO.h"
#include "subsystem/intake/bottomroller/CTREIntakeBottomRollerIO.h"
#include "subsystem/intake/bottomroller/SimIntakeBottomRollerIO.h"
#include "subsystem/intake/deployer/CTREIntakeDeployerIO.h"
#include "subsystem/intake/deployer/SimIntakeDeployerIO.h"
#include "subsystem/intake/toproller/CTREIntakeTopRollerIO.h"
#include "subsystem/intake/toproller/SimIntakeTopRollerIO.h"
#include "subsystem/lights/CTRELightsIO.h"
#include "subsystem/lights/LightsIO.h"
#include "subsystem/vision/LimelightCamera.h"
#include "subsystem/vision/LimelightVisionIO.h"
#include "subsystem/vision/SimVisionIO.h"
#include "utils/MatchTracker.h"

std::unique_ptr<ModuleIO> MakeModuleIO(ModulePosition pos, units::turn_t offset,
                                       CTREModuleIO::DeviceIDs ids) {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimModuleIO>(ModuleConfig{pos, 0_tr});
  }
  return std::make_unique<CTREModuleIO>(ids, ModuleConfig{pos, offset});
}

std::unique_ptr<GyroIO> MakeGyroIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimGyroIO>();
  }
  return std::make_unique<PigeonIO>(HardwareMap::CAN::Pidgeon2::IMU);
}

std::unique_ptr<FlywheelIO> MakeFlywheelIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimFlywheelIO>();
  }
  return std::make_unique<CTREFlywheelIO>(
      HardwareMap::CAN::TalonFX::LeftLeaderFlywheel,
      HardwareMap::CAN::TalonFX::LeftFollowerFlywheel,
      HardwareMap::CAN::TalonFX::RightFollowerFlywheel,
      HardwareMap::CAN::TalonFX::RightBottomFollowerFlywheel);
}

std::unique_ptr<HoodIO> MakeHoodIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimHoodIO>();
  }
  return std::make_unique<CTREHoodIO>(HardwareMap::CAN::TalonFX::HoodMotor);
}

std::unique_ptr<FeederIO> MakeFeederIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimFeederIO>();
  }
  return std::make_unique<CTREFeederIO>(
      HardwareMap::CAN::TalonFX::FeederLeader,
      HardwareMap::CAN::TalonFX::FeederFollower,
      HardwareMap::CAN::TalonFX::FeederFollower2,
      HardwareMap::CAN::TalonFX::FeederFollower3);
}

std::unique_ptr<IntakeDeployerIO> MakeIntakeDeployerIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimIntakeDeployerIO>();
  }
  return std::make_unique<CTREIntakeDeployerIO>(
      HardwareMap::CAN::TalonFX::IntakeDeployer);
}

std::unique_ptr<IntakeTopRollerIO> MakeIntakeTopRollerIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimIntakeTopRollerIO>();
  }
  return std::make_unique<CTREIntakeTopRollerIO>(
      HardwareMap::CAN::TalonFX::IntakeTopRollerLeader,
      HardwareMap::CAN::TalonFX::IntakeTopRollerFollower);
}

std::unique_ptr<IntakeBottomRollerIO> MakeIntakeBottomRollerIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimIntakeBottomRollerIO>();
  }
  return std::make_unique<CTREIntakeBottomRollerIO>(
      HardwareMap::CAN::TalonFX::IntakeBottomRoller);
}

std::unique_ptr<VisionIO> MakeVisionIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimVisionIO>();
  }
  std::vector<LimelightCamera::Config> limelights{
      {"limelight", LimelightCamera::MegaTagMode::kMegaTag1},
      {"limelight-left", LimelightCamera::MegaTagMode::kMegaTag1},
      {"limelight-right", LimelightCamera::MegaTagMode::kMegaTag1},
  };
  return std::make_unique<LimelightVisionIO>(std::move(limelights));
}

std::unique_ptr<LightsIO> MakeLightsIO() {
  return std::make_unique<CTRELightsIO>(HardwareMap::CAN::CANdle::CANdle);
}

RobotContainer::RobotContainer()
    : m_drive{MakeModuleIO(ModulePosition::FrontLeft,
                           Constants::SwerveDrive::kEncoderOffsets[0],
                           {HardwareMap::CAN::TalonFX::FrontLeftDrive,
                            HardwareMap::CAN::TalonFX::FrontLeftSteer,
                            HardwareMap::CAN::CANCoder::FrontLeftEncoder}),
              MakeModuleIO(ModulePosition::FrontRight,
                           Constants::SwerveDrive::kEncoderOffsets[1],
                           {HardwareMap::CAN::TalonFX::FrontRightDrive,
                            HardwareMap::CAN::TalonFX::FrontRightSteer,
                            HardwareMap::CAN::CANCoder::FrontRightEncoder}),
              MakeModuleIO(ModulePosition::BackLeft,
                           Constants::SwerveDrive::kEncoderOffsets[2],
                           {HardwareMap::CAN::TalonFX::BackLeftDrive,
                            HardwareMap::CAN::TalonFX::BackLeftSteer,
                            HardwareMap::CAN::CANCoder::BackLeftEncoder}),
              MakeModuleIO(ModulePosition::BackRight,
                           Constants::SwerveDrive::kEncoderOffsets[3],
                           {HardwareMap::CAN::TalonFX::BackRightDrive,
                            HardwareMap::CAN::TalonFX::BackRightSteer,
                            HardwareMap::CAN::CANCoder::BackRightEncoder}),
              MakeGyroIO()},
      m_flywheel{MakeFlywheelIO()}, m_hood{MakeHoodIO()},
      m_feeder{MakeFeederIO()}, m_intakeDeployer{MakeIntakeDeployerIO()},
      m_intakeTopRoller{MakeIntakeTopRollerIO()},
      m_intakeBottomRoller{MakeIntakeBottomRollerIO()},
      m_vision{MakeVisionIO(), m_drive.GetOdometryThread()},
      m_intake{m_intakeDeployer, m_intakeTopRoller, m_intakeBottomRoller},
      m_shooter{m_flywheel, m_hood}, m_lights{MakeLightsIO()} {
  ConfigureAutoCommands();
  ConfigureBindings();

      m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
  frc::SmartDashboard::PutData("Auto Chooser", &m_autoChooser);

  m_batteryLogger.RegisterSubsystem(
      "Drive", [this] { return m_drive.GetElectricalCurrentDraw(); },
      [this] { return m_drive.GetElectricalPowerDraw(); });
  m_batteryLogger.RegisterSubsystem(
      "Flywheel", [this] { return m_flywheel.GetElectricalCurrentDraw(); },
      [this] { return m_flywheel.GetElectricalPowerDraw(); });
  m_batteryLogger.RegisterSubsystem(
      "Hood", [this] { return m_hood.GetElectricalCurrentDraw(); },
      [this] { return m_hood.GetElectricalPowerDraw(); });
  m_batteryLogger.RegisterSubsystem(
      "Feeder", [this] { return m_feeder.GetElectricalCurrentDraw(); },
      [this] { return m_feeder.GetElectricalPowerDraw(); });
  m_batteryLogger.RegisterSubsystem(
      "IntakeDeployer",
      [this] { return m_intakeDeployer.GetElectricalCurrentDraw(); },
      [this] { return m_intakeDeployer.GetElectricalPowerDraw(); });
  m_batteryLogger.RegisterSubsystem(
      "IntakeTopRoller",
      [this] { return m_intakeTopRoller.GetElectricalCurrentDraw(); },
      [this] { return m_intakeTopRoller.GetElectricalPowerDraw(); });
  m_batteryLogger.RegisterSubsystem(
      "IntakeBottomRoller",
      [this] { return m_intakeBottomRoller.GetElectricalCurrentDraw(); },
      [this] { return m_intakeBottomRoller.GetElectricalPowerDraw(); });
  m_batteryLogger.RegisterAggregateGroup("Intake", [this] {
      return m_intakeTopRoller.GetElectricalCurrentDraw() +
             m_intakeBottomRoller.GetElectricalCurrentDraw() +
             m_intakeDeployer.GetElectricalCurrentDraw();
    },
    [this] {
      return m_intakeTopRoller.GetElectricalPowerDraw() +
             m_intakeBottomRoller.GetElectricalPowerDraw() +
             m_intakeDeployer.GetElectricalPowerDraw();
    });
  m_batteryLogger.RegisterAggregateGroup("Shooter", [this] {
      return m_flywheel.GetElectricalCurrentDraw() +
             m_hood.GetElectricalCurrentDraw();
    },
    [this] {
      return m_flywheel.GetElectricalPowerDraw() +
             m_hood.GetElectricalPowerDraw();
    });
}

void RobotContainer::Periodic() {
  m_batteryLogger.Update();
  m_robotViz.Update();
}

void RobotContainer::ConfigureAutoCommands() {
  pathplanner::NamedCommands::registerCommand(
      "AutoShoot",
      AutoShootCommand(&m_flywheel, &m_hood, &m_feeder, &m_intakeTopRoller,
                       &m_intakeBottomRoller, &m_intakeDeployer)
          .ToPtr());

  pathplanner::NamedCommands::registerCommand(
      "Shoot", ShootCommand(
                   &m_drive, &m_flywheel, &m_hood, &m_intakeTopRoller,
                   &m_intakeBottomRoller, &m_feeder, &m_intakeDeployer,
                   [this] { return -m_driver.GetLeftY(); },
                   [this] { return -m_driver.GetLeftX(); })
                   .ToPtr());

  pathplanner::NamedCommands::registerCommand(
      "Intake", IntakeCommand(&m_drive, &m_intakeDeployer, &m_intakeTopRoller,
                              &m_intakeBottomRoller)
                    .ToPtr());

  pathplanner::NamedCommands::registerCommand(
      "StopIntake", StopIntakeCommand(&m_intakeDeployer, &m_intakeTopRoller,
                                      &m_intakeBottomRoller)
                        .ToPtr());

  pathplanner::NamedCommands::registerCommand(
      "SpinUpFlywheel", ShotCalculatedSpinUpCommand(&m_flywheel).ToPtr());
}

void RobotContainer::ConfigureBindings() {
  using frc2::cmd::Run;
  using frc2::cmd::RunOnce;

  m_flywheel.SetDefaultCommand(
      ShotCalculatedSpinUpCommand(&m_flywheel).ToPtr());

  m_drive.SetDefaultCommand(DriveMaintainingHeadingCommand(
      &m_drive, [this] { return -m_driver.GetLeftY(); },
      [this] { return -m_driver.GetLeftX(); },
      [this] { return -m_driver.GetRightX() * 0.5; },
      [this] { return m_driver.Options().Get(); }, true));

  m_driver.POVUp().WhileTrue(
      Run([this] { m_hood.SetPosition(0_deg); }, {&m_hood}));

  m_driver.L2().WhileTrue(IntakeCommand(&m_drive, &m_intakeDeployer,
                                        &m_intakeTopRoller,
                                        &m_intakeBottomRoller)
                              .ToPtr());

  m_driver.Options().WhileTrue(Run([this] { m_drive.ResetHeading(0_deg); }));

  m_driver.R2().WhileTrue(ShootCommand(
                              &m_drive, &m_flywheel, &m_hood,
                              &m_intakeTopRoller, &m_intakeBottomRoller,
                              &m_feeder, &m_intakeDeployer,
                              [this] { return -m_driver.GetLeftY(); },
                              [this] { return -m_driver.GetLeftX(); })
                              .ToPtr());

  m_driver.R1().WhileTrue(EjectIntakeCommand(&m_intakeDeployer,
                                             &m_intakeTopRoller,
                                             &m_intakeBottomRoller, &m_feeder)
                              .ToPtr());
  // m_driver.R1().WhileTrue(SimpleShootCommand(&m_flywheel, &m_feeder,
  //                                            &m_hood, &m_intakeBottomRoller,
  //                                            &m_intakeDeployer, 1000_rpm,
  //                                            60_tps, 10_deg)
  //                             .ToPtr());

  m_driver.POVLeft().OnFalse(RunOnce(
      [this] { m_intakeDeployer.ZeroPosition(); }, {&m_intakeDeployer}));

  m_driver.Cross().WhileTrue(
      frc2::cmd::Defer(
          [this]() {
            return m_drive.GetPathCommand(
                RobotState::Instance()
                    .GetDriveState(frc::Timer::GetFPGATimestamp())
                    .estimatedPose);
          },
          {&m_drive})
          .AlongWith(Run([this] { m_hood.SetPosition(0_deg); }, {&m_hood}))
          .AlongWith(Run([this] { m_intakeDeployer.RetractMid(); },
                         {&m_intakeDeployer})));

  m_driver.L1().WhileTrue(
          AlignToEdgeCommand(&m_drive,
                              [this] { return -m_driver.GetLeftY(); },
                              [this] { return -m_driver.GetLeftX(); })
                              .ToPtr().AlongWith(
        IntakeCommand(&m_drive, &m_intakeDeployer, &m_intakeTopRoller,
                              &m_intakeBottomRoller)
                    .ToPtr()
        ));

    /* -------------------- OPERATOR CONTROLLER COMMAND -------------------- */

  m_operator.R1().OnTrue(
      RunOnce([] {
        MatchTracker::Instance().SetAutoWinnerFromCurrentAlliance(true);
      }).IgnoringDisable(true));
  m_operator.L1().OnTrue(
      RunOnce([] {
        MatchTracker::Instance().SetAutoWinnerFromCurrentAlliance(false);
      }).IgnoringDisable(true));
  /* -------------------- DEBUG CONTROLLER COMMAND -------------------- */
  // m_debugger.Cross().OnTrue(
  //       m_flywheel.SysIdDynamic(frc2::sysid::Direction::kForward)
  //);
  // m_debugger.Circle().OnTrue(
  //       m_flywheel.SysIdDynamic(frc2::sysid::Direction::kReverse)
  //);
  // m_debugger.Triangle().OnTrue(
  //       m_flywheel.SysIdQuasistatic(frc2::sysid::Direction::kForward)
  //);

  // m_debugger.Square().OnTrue(
  //       m_flywheel.SysIdQuasistatic(frc2::sysid::Direction::kReverse)
  //);

  // m_debugger.Cross().OnTrue(
  //        m_feeder.SysIdDynamic(frc2::sysid::Direction::kForward)
  // );
  //  m_debugger.Circle().OnTrue(
  //        m_feeder.SysIdDynamic(frc2::sysid::Direction::kReverse)
  // );
  //  m_debugger.Triangle().OnTrue(
  //        m_feeder.SysIdQuasistatic(frc2::sysid::Direction::kForward)
  // );

  //  m_debugger.Square().OnTrue(
  //        m_feeder.SysIdQuasistatic(frc2::sysid::Direction::kReverse)
  // );
  // m_debugger.Cross().WhileTrue(SimpleShootCommand(&m_flywheel, &m_feeder,
  //                                            &m_hood, &m_intakeBottomRoller,
  //                                            &m_intakeDeployer, 1000_rpm,
  //                                            60_tps, 10_deg)
  //                             .ToPtr());
}

frc2::Command *RobotContainer::GetAutonomousCommand() {
  return m_autoChooser.GetSelected();
}

void RobotContainer::SetAutoDriveCurrentLimits() {
  m_drive.SetAutoCurrentLimits();
}

void RobotContainer::SetTeleopDriveCurrentLimits() {
  m_drive.SetTeleopCurrentLimits();
}
