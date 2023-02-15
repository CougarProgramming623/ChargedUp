// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Pose2d.h>
#include "Util.h"
#include "commands/TrajectoryCommand.h"
#include <frc/kinematics/SwerveModulePosition.h>

using namespace pathplanner;

Robot* Robot::s_Instance = nullptr;

void Robot::RobotInit() {
  GetNavX().ZeroYaw();
  s_Instance = this;
  m_DriveTrain.DriveInit();
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
  //GetDriveTrain().m_Odometry.update;

  
  DebugOutF("Auto init");

  frc2::CommandScheduler::GetInstance().CancelAll();
  GetNavX().ZeroYaw();
  GetNavX().SetAngleAdjustment(-90);

  GetDriveTrain().m_FrontLeftModule.m_DriveController.motor.SetSelectedSensorPosition(0);
  GetDriveTrain().m_FrontRightModule.m_DriveController.motor.SetSelectedSensorPosition(0);
  GetDriveTrain().m_BackLeftModule.m_DriveController.motor.SetSelectedSensorPosition(0);
  GetDriveTrain().m_BackRightModule.m_DriveController.motor.SetSelectedSensorPosition(0);

  for(int i = 0; i < 4; i++){
    GetDriveTrain().m_ModulePositions[i] = frc::SwerveModulePosition(0_m, frc::Rotation2d(0_rad));
  }

  GetDriveTrain().GetOdometry().ResetPosition(units::radian_t(0), GetDriveTrain().GetModulePositions(), frc::Pose2d(5_m, 4_m, 0_rad));
  //GetDriveTrain().GetOdometry().ResetPosition(units::radian_t(Deg2Rad(-fmod(360 - 180 + 90 - Robot::s_Instance->GetNavX().GetAngle(), 360))), GetDriveTrain().GetModulePositions(), frc::Pose2d(0_m, 0_m, 0_rad)); //uncomment this

  GetDriveTrain().BreakMode(true);

  //Load trajectory
  PathPlannerTrajectory traj = PathPlanner::loadPath("StraightLine", PathConstraints(2_mps, 2_mps_sq));

  //traj = traj.transformTrajectoryForAlliance();

  //Match starting pose to trajectory pose, pose becomes field relative
  //GetDriveTrain().GetOdometry().GetPose().RelativeTo();
  //GetDriveTrain().GetOdometry().ResetPosition(GetDriveTrain().GetRotation(),
  /*frc::Rotation2d(0_rad)*///, 

  //wpi::array<frc::SwerveModulePosition,4>(
  //   frc::SwerveModulePosition(0_m, frc::Rotation2d(0_rad)),
  //   frc::SwerveModulePosition(0_m, frc::Rotation2d(0_rad)),
  //   frc::SwerveModulePosition(0_m, frc::Rotation2d(0_rad)),
  //   frc::SwerveModulePosition(0_m, frc::Rotation2d(0_rad)))
  //GetDriveTrain().GetModulePositions(), frc::Pose2d(0_m, 0_m, 0_rad));
  //


  // DebugOutF("InitialRotation: " + std::to_string(traj.getInitialHolonomicPose().Rotation().Degrees().value()));
  // DebugOutF("InitialY: " + std::to_string(traj.asWPILibTrajectory().InitialPose().Y().value()));
  // DebugOutF("InitialX: " + std::to_string(traj.asWPILibTrajectory().InitialPose().X().value()));

  //frc2::CommandScheduler::GetInstance().Schedule(new TrajectoryCommand(traj));
}

void Robot::AutonomousPeriodic() {
  // int i = 0;
  // if(i % 100 == 0){
    DebugOutF("X: " + std::to_string(GetDriveTrain().GetOdometry().GetPose().X().value()));
    DebugOutF("Y: " + std::to_string(GetDriveTrain().GetOdometry().GetPose().Y().value()));
    DebugOutF("Deg: " + std::to_string(GetDriveTrain().GetOdometry().GetPose().Rotation().Degrees().value()));
  //   i = 0;
  // }
  // i++;
  
}

void Robot::TeleopInit() {

  GetNavX().ZeroYaw();
  GetNavX().SetAngleAdjustment(-90);
   
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand 
    = nullptr;
  }

  //Standard teleop pose init
  //GetDriveTrain().GetOdometry().ResetPosition(frc::Rotation2d(units::radian_t(0)), GetDriveTrain().GetModulePositions(), frc::Pose2d());
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  
  frc2::CommandScheduler::GetInstance().Run();

  //DebugOutF(std::to_string(Deg2Rad(360-(fmod(((GetDriveTrain().m_BackRightModule.GetSteerController().encoder.GetVoltage() * ENCODER_VOLTAGE_TO_DEGREE) + (360+2)), 360))) / STEER_ENCODER_POSITION_CONSTANT));
  DebugOutF("X: " + std::to_string(GetDriveTrain().GetOdometry().GetPose().X().value()));
  DebugOutF("Y: " + std::to_string(GetDriveTrain().GetOdometry().GetPose().Y().value()));
  DebugOutF("Deg: " + std::to_string(GetDriveTrain().GetOdometry().GetPose().Rotation().Degrees().value()) + "/n");
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
