// Team 5687 2026

#include "commands/shooter/ShootCommand.h"

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <algorithm>
#include <numbers>

#include "subsystem/feeder/FeederConstants.h"
#include "utils/Logger.h"
#include "utils/Utils.h"

ShootCommand::ShootCommand(DriveSubsystem *drive, FlywheelSubsystem *flywheel,
                           HoodSubsystem *hood,
                           IntakeTopRollerSubsystem *topRoller,
                           IntakeBottomRollerSubsystem *bottomRoller,
                           FeederSubsystem *feeder, FloorSubsystem *floor,
                           IntakeDeployerSubsystem *deployer,
                           std::function<double()> throttle,
                           std::function<double()> strafe)
    : m_drive(drive), m_flywheel(flywheel), m_hood(hood),
      m_topRoller(topRoller), m_bottomRoller(bottomRoller), m_feeder(feeder),
      m_floor(floor), m_deployer(deployer), m_throttle(throttle),
      m_strafe(strafe) {
  AddRequirements({drive, flywheel, hood, feeder, floor, deployer});
  SetName("ShootCommand");
  m_headingController.EnableContinuousInput(-std::numbers::pi,
                                            std::numbers::pi);
  m_headingController.SetTolerance(0.035); // ~2 deg
}

void ShootCommand::ResetSequenceState() {
  m_phase = Phase::kIdle;
  m_phaseStartTime = 0_s;
}

void ShootCommand::EnterPhase(Phase phase, units::second_t now) {
  m_phase = phase;
  m_phaseStartTime = now;
}

double ShootCommand::BurstOffset(units::second_t now) const {
  if (!kEnableBurst) {
    return 0.0;
  }
  if (m_phase != Phase::kExtendHold) {
    return 0.0;
  }
  auto elapsed = now - m_phaseStartTime;
  if (elapsed < kFeederStartDelay) {
    return kBurstRpmOffset;
  }
  auto rampElapsed = elapsed - kFeederStartDelay;
  if (rampElapsed >= kBurstRampDuration) {
    return 0.0;
  }
  double frac = rampElapsed / kBurstRampDuration;
  return kBurstRpmOffset * (1.0 - frac);
}

bool ShootCommand::FeedersShouldRun(units::second_t now) const {
  if (m_phase == Phase::kIdle || m_phase == Phase::kPreClear) {
    return false;
  }
  if (m_phase == Phase::kExtendHold &&
      (now - m_phaseStartTime) < kFeederStartDelay) {
    return false;
  }
  return true;
}

void ShootCommand::Initialize() {
  m_headingController.Reset();
  m_drive->SetMaxSpeeds(
      Constants::SwerveDrive::Shooting::kMaxSpeedsWhileShooting);
  ResetSequenceState();
  m_hasFedFuel = false;
  m_feeder->SetIndexingActive(false);
  // Preclear only if indexing is still needed.
  if (m_feeder->NeedsIndexing()) {
    auto now = frc::Timer::GetFPGATimestamp();
    EnterPhase(Phase::kPreClear, now);
    m_feeder->BeginClearance();
  }
}

void ShootCommand::UpdateFlywheelAndHood(const ShotSolution &solution,
                                         units::second_t now) {
  // Keep fuel off the wheel during preclear.
  if (m_phase == Phase::kPreClear) {
    m_flywheel->SetVoltage(kPreclearFlywheelReverseVoltage);
    m_hood->SetPosition(1_deg);
    return;
  }
  double targetRPM = solution.flywheelSpeed + BurstOffset(now);
  m_flywheel->SetRPM(units::revolutions_per_minute_t{targetRPM});
  m_hood->SetPosition(units::radian_t{solution.hoodAngle});
}

void ShootCommand::UpdateDrive(const ShotSolution &solution) {
  double throttle = ApplyDeadband(m_throttle(), kDeadband);
  double strafe = ApplyDeadband(m_strafe(), kDeadband);

  auto maxSpeeds = m_drive->GetMaxSpeeds();
  units::meters_per_second_t maxLinearSpeed = maxSpeeds.first;
  auto xVel = throttle * maxLinearSpeed;
  auto yVel = strafe * maxLinearSpeed;

  double rotOutput =
      m_headingController.Calculate(m_drive->GetOdometryThread()
                                        ->GetEstimatedPose()
                                        .Rotation()
                                        .Radians()
                                        .value(),
                                    solution.driveAngle.Radians().value());
  rotOutput =
      std::clamp(rotOutput, -Constants::SwerveDrive::kMaxAngularSpeed.value(),
                 Constants::SwerveDrive::kMaxAngularSpeed.value());

  m_drive->DriveFieldRelative(
      frc::ChassisSpeeds{xVel, yVel, units::radians_per_second_t{rotOutput}});
}

void ShootCommand::UpdateSequence(units::second_t now, bool solutionReady) {
  auto elapsed = now - m_phaseStartTime;
  switch (m_phase) {
  case Phase::kIdle:
    if (solutionReady) {
      EnterPhase(Phase::kExtendHold, now);
      m_deployer->FullyExtend();
    }
    break;
  case Phase::kPreClear:
    if (m_feeder->IsCleared() ||
        elapsed >= Constants::Feeder::kClearanceTimeout) {
      if (!m_feeder->IsCleared()) {
        Logger::Instance().Log("ShootCommand/ClearTimeout", true);
      }
      m_feeder->SetIndexed();
      m_feeder->Stop();
      m_floor->Stop();
      EnterPhase(Phase::kIdle, now);
    } else {
      m_floor->SetVoltage(kBackoffFloorVoltage);
    }
    break;
  case Phase::kExtendHold:
    if (elapsed >= kInitialExtendDuration) {
      EnterPhase(Phase::kQuickRetract, now);
      m_deployer->SlowRetract(kPulseRetractDuration);
    }
    break;
  case Phase::kQuickRetract:
    if (elapsed >= kPulseRetractDuration) {
      EnterPhase(Phase::kReExtend, now);
      m_deployer->FullyExtend();
    }
    break;
  case Phase::kReExtend:
    if (m_deployer->IsFullyExtended()) {
      EnterPhase(Phase::kSlowRetract, now);
      m_deployer->SlowRetract(kFinalRetractDuration);
    }
    break;
  case Phase::kSlowRetract:
    break;
  }
}

void ShootCommand::RunFeeders() {
  if (!m_hasFedFuel) {
    m_hasFedFuel = true;
    // Re-arm indexing as soon as a feed starts.
    m_feeder->ClearIndexed();
  }
  m_floor->SetVoltage(kFloorVoltage);
  m_feeder->SetVoltage(kFeederVoltage);
  m_topRoller->SetVoltage(kTopVoltage);
  m_bottomRoller->SetVoltage(kBottomVoltage);
}

void ShootCommand::Execute() {
  auto now = frc::Timer::GetFPGATimestamp();
  bool isRed =
      frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;

  auto solution = m_shotCalculator.Calculate(now, isRed);

  UpdateSequence(now, solution.ready);
  UpdateFlywheelAndHood(solution, now);
  UpdateDrive(solution);

  if (FeedersShouldRun(now)) {
    RunFeeders();
  }

  auto &log = Logger::Instance();
  log.Log("ShootCommand/Phase", static_cast<int>(m_phase));
  log.Log("ShootCommand/SolutionReady", solution.ready);
  log.Log("ShootCommand/BurstOffset", BurstOffset(now));
}

void ShootCommand::End(bool interrupted) {
  m_flywheel->SetRPM(0_rpm);
  m_hood->SetPosition(0_rad);
  m_topRoller->Stop();
  m_bottomRoller->Stop();
  m_feeder->Stop();
  m_floor->Stop();
  m_deployer->RetractMid();
  ResetSequenceState();
  m_drive->SetMaxSpeeds(Constants::SwerveDrive::kMaxLinearSpeed);
  m_drive->Stop();
}

bool ShootCommand::IsFinished() { return false; }
