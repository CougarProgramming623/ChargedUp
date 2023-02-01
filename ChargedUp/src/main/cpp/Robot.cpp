// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Pose2d.h>
#include "Util.h"

using namespace pathplanner;

Robot* Robot::s_Instance = nullptr;

void Robot::RobotInit() {
  GetNavX().ZeroYaw();
  s_Instance = this;
  m_DriveTrain.DriveInit();
  m_AutoTimer = frc::Timer();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_AutoTimer.Start();
  GetDriveTrain().m_Odometry.ResetPosition(frc::Rotation2d(units::radian_t(0)), GetDriveTrain().m_ModulePositions, frc::Pose2d(frc::Translation2d(2_m, 3_m), frc::Rotation2d(units::radian_t(0))));
  GetDriveTrain().TrajectoryFollow(
    PathPlanner::loadPath("Test2", PathConstraints(.5_mps, .5_mps_sq)).asWPILibTrajectory()//,
    // PathPlanner::loadPath("Test1", PathConstraints(4_mps, 3_mps_sq)).sample(m_AutoTimer.Get() + 0.1_s).holonomicRotation
  );

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
   
}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand 
    = nullptr;
  }
  //GetDriveTrain().m_Odometry.ResetPosition(0, 0, 0);
  GetDriveTrain().m_Odometry.ResetPosition(frc::Rotation2d(units::radian_t(0)), GetDriveTrain().m_ModulePositions, frc::Pose2d());
  GetNavX().ZeroYaw();
  GetNavX().SetAngleAdjustment(-90);
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  // DebugOutF("X: " + std::to_string(GetDriveTrain().m_Odometry.GetPose().X().value()));
  // DebugOutF("Y: " + std::to_string(GetDriveTrain().m_Odometry.GetPose().Y().value()));
  // DebugOutF("Deg: " + std::to_string(GetDriveTrain().m_Odometry.GetPose().Rotation().Degrees().value()) + "/n");
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
