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
#include <frc/RobotController.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/SequentialCommandGroup.h>


using ctre::phoenix::motorcontrol::ControlMode;
using namespace pathplanner;

Robot* Robot::s_Instance = nullptr;

Robot::Robot() :
m_NavX(frc::SerialPort::Port(2), AHRS::SerialDataType(0), uint8_t(66)),
m_LED(),
m_Arm()
{
  s_Instance = this;
}

void Robot::RobotInit() {

  GetNavX().ZeroYaw();
  GetNavX().SetAngleAdjustment(0);
  s_Instance = this;
  m_DriveTrain.DriveInit();
  m_Vision.VisionInit(); //Make one
  m_LED.Init();
  m_Arm.ArmInit();

  
  AutoButtons();
  m_LED.Init();
  
  m_COBTicks = 0;
  m_AutoPath = "";
  m_AutoFlag = true;

  if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
    m_ColOffset = 2;
  } else {
    m_ColOffset = 0;
  }
}

void Robot::AutoButtons(){

  //BUTTONBOARD 2
  m_TL          = frc2::Button(BUTTON_L_TWO(GRID_TL));
  m_TC          = frc2::Button(BUTTON_L_TWO(GRID_TC));
  m_TR          = frc2::Button(BUTTON_L_TWO(GRID_TR));
  m_ML          = frc2::Button(BUTTON_L_TWO(GRID_ML));
  m_MC          = frc2::Button(BUTTON_L_TWO(GRID_MC));
  m_MR          = frc2::Button(BUTTON_L_TWO(GRID_MR));
  m_BL          = frc2::Button(BUTTON_L_TWO(GRID_BL));
  m_BC          = frc2::Button(BUTTON_L_TWO(GRID_BC));
  m_BR          = frc2::Button(BUTTON_L_TWO(GRID_BR));
  m_BigRed      = frc2::Button(BUTTON_L(BIG_RED));

  m_SingleSub   = frc2::Button(BUTTON_L(5));
  m_DoubleSub = frc2::Button(BUTTON_L_TWO(13));
  m_SingleSubCube = frc2::Button(BUTTON_L(7));

m_LeftGrid    = frc2::Button(BUTTON_L_TWO(LEFT_GRID));q@2Q1
  m_CenterGrid  = frc2::Button(BUTTON_L_TWO(CENTER_GRID));
  m_RightGrid   = frc2::Button(BUTTON_L_TWO(RIGHT_GRID));

  // m_MidCone = frc2::Button(BUTTON_L_TWO(TRANSIT_MODE));
	// m_MidCube = frc2::Button(BUTTON_L_TWO(GROUND_PICKUP_MODE));
  m_PlacingMode = frc2::Button(BUTTON_L_TWO(PLACING_MODE));
  m_GroundPickup = frc2::Button(BUTTON_L_TWO(GROUND_PICKUP_MODE));

  m_NavXReset = frc2::Button(BUTTON_L(8)); //PUT Define
  m_AutoBalance = frc2::Button(BUTTON_L(3));
   m_Print = frc2::Button(BUTTON_L(2));

  m_AutoBalance.WhileHeld(new AutoBalance());
  m_VisionPoseReset = frc2::Button([&] { return Robot::GetRobot()->GetButtonBoard().GetRawButton(6); }); //PUT Define
 
  
  m_Print.WhileHeld(
    new frc2::InstantCommand([&]{
      
    })
  );

  
  
  m_NavXReset.WhenPressed(
    new frc2::InstantCommand([&]{
      DebugOutF("NavX Zero");
      zeroGyroscope();
  }));
  
 

  m_TR.WhenPressed(frc2::PrintCommand("Nothing"));

  // m_GroundPickup.WhenPressed( //redefine in .h
    
  // );

  // m_SingleSub.WhenPressed( //redefine in .h
    
  // );

  // m_DoubleSub.WhenPressed( //redefine in .h
    
  // );

  // m_SingleSubCube.WhenPressed( //redefine in .h
      
  // );

  m_ML.WhenPressed(new frc2::InstantCommand([&]{
    
  }));


  m_BL.WhenPressed(new frc2::InstantCommand([&]{
    
  }));


  m_TC.WhenPressed(new frc2::InstantCommand([&]{
    
  }));


  m_MC.WhenPressed(new frc2::InstantCommand([&]{
    DebugOutF("m_MC");
    SelectedRow = 1;
    SelectedColumn = std::abs(m_ColOffset - 1);
    frc::Pose2d SelectedPose = 
		  Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));

  m_BC.WhenPressed(new frc2::InstantCommand([&]{
    DebugOutF("m_BC");
    SelectedRow = 2;
    SelectedColumn = std::abs(m_ColOffset - 1);
    frc::Pose2d SelectedPose = 
		  Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));


  // m_TR.WhenPressed(new frc2::InstantCommand([&]{
  //   DebugOutF("m_TR - nothing");
  //   // frc::Pose2d SelectedPose = 
	// 	// Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
	// 	// Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  // }));

  m_MR.WhenPressed(new frc2::InstantCommand([&]{
		
  }));

  m_BR.WhenPressed(new frc2::InstantCommand([&]{
		DebugOutF("m_BR");
    SelectedRow = 2;
    SelectedColumn = std::abs(m_ColOffset - 2);
		frc::Pose2d SelectedPose = 
		  Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));

  // m_BigRed.WhenPressed( //redefine in .h
    
	// );

  m_LeftGrid.WhenPressed(new frc2::InstantCommand([&]{
    DebugOutF("m_LeftGrid");
		if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)) {
      Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 0;
		} else{
			Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 2;
	}}));

  m_CenterGrid.WhenPressed(new frc2::InstantCommand([&]{
    Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 1;
  }));

  m_RightGrid.WhenPressed(new frc2::InstantCommand([&]{
		DebugOutF("m_RightGrid");
		if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
			Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 2;
		} else{
			Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 0;
	}}));

  m_VisionPoseReset.WhenPressed(
    new frc2::InstantCommand([&] {
      DebugOutF("Pose Resetting");
      if(COB_GET_ENTRY(m_Vision.FrontBack("tv")).GetInteger(0) == 1 && COB_GET_ENTRY(m_Vision.FrontBack("botpose")).GetDoubleArray(std::span<double>()).size() != 0){
        frc::Pose2d startingPose = frc::Pose2d(m_Vision.GetPoseBlue().Translation(), units::radian_t(Deg2Rad(GetAngle())));
        GetDriveTrain().GetOdometry()->ResetPosition(units::radian_t(Deg2Rad(GetAngle())), 
        wpi::array<frc::SwerveModulePosition, 4>
              (GetDriveTrain().m_FrontLeftModule.GetPosition(), GetDriveTrain().m_FrontRightModule.GetPosition(), GetDriveTrain().m_BackLeftModule.GetPosition(), GetDriveTrain().m_BackRightModule.GetPosition()), 
        startingPose);
        DebugOutF("Pose Reset. X: " + std::to_string(startingPose.X().value()) + ", Y: " + std::to_string(startingPose.Y().value()) + ", Z: " + std::to_string(startingPose.Rotation().Degrees().value()));
      } else {
        DebugOutF("Pose Reset Fail");
      }
    })
  );
}

frc::Pose2d Robot::TransformPose(frc::Pose2d SelectedPose){ //rotating poses do not add correctly
	if(Robot::GetRobot()->GetDriveTrain().m_SelectedGrid == 1){
		SelectedPose = SelectedPose +
			frc::Transform2d(
				frc::Translation2d(units::meter_t(0), units::meter_t(1.68)),
				frc::Rotation2d(units::radian_t(0))
		).Inverse(); //delete inverse if not going 180
	} else if(Robot::GetRobot()->GetDriveTrain().m_SelectedGrid == 2){
		SelectedPose = SelectedPose + 
			frc::Transform2d(
				frc::Translation2d(units::meter_t(0), units::meter_t(2 * 1.68)),
				frc::Rotation2d(units::radian_t(0))
		).Inverse(); //delete inverse if not going 180	
	}
	if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
		SelectedPose = 
			frc::Pose2d(
				units::meter_t(16.541)-SelectedPose.Translation().X(), 
				SelectedPose.Translation().Y(),
				SelectedPose.Rotation().RotateBy(Rotation2d(units::degree_t(180)))
			);
	}
	return SelectedPose;
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  Robot::GetCOB().GetTable().GetEntry("/COB/robotAngle").SetDouble(Robot::GetAngle());   
  Robot::GetCOB().GetTable().GetEntry("/COB/matchTime").SetDouble(DriverStation::GetMatchTime());
  Robot::GetCOB().GetTable().GetEntry("/COB/ticks").SetDouble(m_COBTicks);
  Robot::GetCOB().GetTable().GetEntry("/COB/deltaX").SetDouble(std::abs(GetDriveTrain().m_VisionRelative.X().value()));
  Robot::GetCOB().GetTable().GetEntry("/COB/deltaY").SetDouble(std::abs(GetDriveTrain().m_VisionRelative.Y().value()));
  Robot::GetCOB().GetTable().GetEntry("/COB/deltaT").SetDouble(std::abs(-fmod(360 - GetDriveTrain().m_VisionRelative.Rotation().Degrees().value(), 360)));
  m_Vision.PushID();

  m_COBTicks++;
  Robot::GetRobot()->GetCOB().GetTable().GetEntry("/COB/pitchAngle").SetDouble(Robot::GetRobot()->GetNavX().GetPitch() + 0.05);
  m_AutoPath = std::string(Robot::GetRobot()->GetCOB().GetTable().GetEntry("/COB/auto").GetString(""));
  // DebugOutF("Row: " + std::to_string(SelectedRow) + " , Col: " + std::to_string(SelectedColumn));


  if(Robot::GetButtonBoard().GetRawButton(2)){
    
  }

  if(Robot::GetButtonBoard().GetRawButton(4)){
    DebugOutF("BL: " + std::to_string(Rad2Deg(GetDriveTrain().m_BackLeftModule.GetSteerAngle())));
    DebugOutF("BR: " + std::to_string(Rad2Deg(GetDriveTrain().m_BackRightModule.GetSteerAngle())));
    DebugOutF("FL: " + std::to_string(Rad2Deg(GetDriveTrain().m_FrontLeftModule.GetSteerAngle())));
    DebugOutF("FR: " + std::to_string(Rad2Deg(GetDriveTrain().m_FrontRightModule.GetSteerAngle())));
  }

  //LED
  m_LED.SponsorBoardAllianceColor();
  //m_LED.LowBattery();
  m_LED.EyesAllianceColor();
  m_LED.EndGame();
  m_LED.SetData();                                                                                                                                   
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  GetDriveTrain().BreakMode(true);
  GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

  
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {  
  m_AutoFlag = true;
  DebugOutF("Auto init");

  frc2::CommandScheduler::GetInstance().CancelAll();
  GetNavX().ZeroYaw();
  GetNavX().SetAngleAdjustment(0);
  GetDriveTrain().BreakMode(true);
  GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  //PathPlannerTrajectory traj;

  //Load trajectory
  // if(!COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
    // DebugOutF("Blue");
  //PathPlannerTrajectory traj = PathPlanner::loadPath(m_AutoPath, PathConstraints(4_mps, 1_mps_sq));

  PathPlannerTrajectory traj = PathPlanner::loadPath("AutoBalanceExtra", PathConstraints(4_mps, 1.5_mps_sq));

  // } else {
  // //   DebugOutF("Red");
  // //   PathPlannerTrajectory traj = PathPlanner::loadPath("TestBalanceRed", PathConstraints(4_mps, 1_mps_sq));
  // // }


  //PathPlannerTrajectory::transformTrajectoryForAlliance(traj, frc::DriverStation::GetAlliance());

  frc::Pose2d startingPose = frc::Pose2d(traj.getInitialState().pose.Translation(), frc::Rotation2d(units::degree_t(0)));

  GetDriveTrain().GetOdometry()->ResetPosition(units::radian_t(Deg2Rad(GetAngle())), 
    wpi::array<frc::SwerveModulePosition, 4>
         (GetDriveTrain().m_FrontLeftModule.GetPosition(), GetDriveTrain().m_FrontRightModule.GetPosition(), GetDriveTrain().m_BackLeftModule.GetPosition(), GetDriveTrain().m_BackRightModule.GetPosition()), 
    startingPose);
  
  
  // DebugOutF("InitialRotation: " + std::to_string(traj.getInitialHolonomicPose().Rotation().Degrees().value()));
  // DebugOutF("InitialY: " + std::to_string(traj.asWPILibTrajectory().InitialPose().Y().value()));
  // DebugOutF("InitialX: " + std::to_string(traj.asWPILibTrajectory().InitialPose().X().value()));
  if(true/*COB_GET_ENTRY("/COB/autos").GetString("") == "Auto1"*/){
    frc2::CommandScheduler::GetInstance().Schedule(
      new frc2::SequentialCommandGroup(
      AutoBalance()
  ));

  }
}
void Robot::AutonomousPeriodic() {

    // DebugOutF("X: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().X().value()));
    // DebugOutF("Y: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Y().value()));
    // DebugOutF("Deg: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Rotation().Degrees().value()));
  
}

void Robot::TeleopInit() {
  m_AutoFlag = false;
  frc2::CommandScheduler::GetInstance().Schedule(new frc2::InstantCommand([&] {
      if(COB_GET_ENTRY(m_Vision.FrontBack("tv")).GetInteger(0) == 1 && COB_GET_ENTRY(m_Vision.FrontBack("botpose")).GetDoubleArray(std::span<double>()).size() != 0){
        frc::Pose2d startingPose = frc::Pose2d(m_Vision.GetPoseBlue().Translation(), units::radian_t(Deg2Rad(GetAngle())));
        GetDriveTrain().GetOdometry()->ResetPosition(units::radian_t(Deg2Rad(GetAngle())), 
        wpi::array<frc::SwerveModulePosition, 4>
              (GetDriveTrain().m_FrontLeftModule.GetPosition(), GetDriveTrain().m_FrontRightModule.GetPosition(), GetDriveTrain().m_BackLeftModule.GetPosition(), GetDriveTrain().m_BackRightModule.GetPosition()), 
        startingPose);
        DebugOutF("Pose Reset. X: " + std::to_string(startingPose.X().value()) + ", Y: " + std::to_string(startingPose.Y().value()) + ", Z: " + std::to_string(startingPose.Rotation().Degrees().value()));
      } else {
        DebugOutF("Pose Reset Fail");
      }
    })
  );
  m_LED.m_IsTele = true;  // used for LED Timer
  //GetNavX().ZeroYaw();
  m_DriveTrain.BreakMode(true);
  GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetNavX().SetAngleAdjustment(0);
   
  // frc::Pose2d startingPose = frc::Pose2d(units::meter_t(2.54), units::meter_t(1.75), frc::Rotation2d(units::degree_t(0)));
  //   GetDriveTrain().GetOdometry()->ResetPosition(units::radian_t(Deg2Rad(GetAngle())), 
  //       wpi::array<frc::SwerveModulePosition, 4>
  //           (GetDriveTrain().m_FrontLeftModule.GetPosition(), GetDriveTrain().m_FrontRightModule.GetPosition(), GetDriveTrain().m_BackLeftModule.GetPosition(), GetDriveTrain().m_BackRightModule.GetPosition()), 
  //       startingPose);
}

/**
 * This function is called periodically during operator control.  
 */
void Robot::TeleopPeriodic() {
  //DebugOutF("Theta: " + std::to_string(GetAngle())); 
  
  // DebugOutF("LLX: " + std::to_string(m_Vision.GetPoseBlue().X().value()));
  // DebugOutF("LLY: " + std::to_string(m_Vision.GetPoseBlue().Y().value()));
  // DebugOutF("LLZ: " + std::to_string(m_Vision.GetPoseBlue().Rotation().Degrees().value()));
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
