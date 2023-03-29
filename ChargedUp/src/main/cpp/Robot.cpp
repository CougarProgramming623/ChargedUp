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

using ctre::phoenix::motorcontrol::ControlMode;
using namespace pathplanner;

Robot* Robot::s_Instance = nullptr;

Robot::Robot()
{
  s_Instance = this;
}

void Robot::RobotInit() {

  GetNavX().ZeroYaw();
  GetNavX().SetAngleAdjustment(0);
  s_Instance = this;
  m_DriveTrain.DriveInit();
  m_Vision.VisionInit(); //Make one
  AutoButtons();
  m_Arm.Init();
  m_COBTicks = 0;
  m_AutoPath = "";
}

void Robot::AutoButtons(){

  //BUTTONBOARD 2
  m_TL = frc2::Button(BUTTON_L_TWO(GRID_TL));
  m_TC = frc2::Button(BUTTON_L_TWO(GRID_TC));
  m_TR = frc2::Button(BUTTON_L_TWO(GRID_TR));
  m_ML = frc2::Button(BUTTON_L_TWO(GRID_ML));
  m_MC = frc2::Button(BUTTON_L_TWO(GRID_MC));
  m_MR = frc2::Button(BUTTON_L_TWO(GRID_MR));
  m_BL = frc2::Button(BUTTON_L_TWO(GRID_BL));
  m_BC = frc2::Button(BUTTON_L_TWO(GRID_BC));
  m_BR = frc2::Button(BUTTON_L_TWO(GRID_BR));

  m_LeftGrid = frc2::Button(BUTTON_L_TWO(LEFT_GRID));
  m_CenterGrid = frc2::Button(BUTTON_L_TWO(CENTER_GRID));
  m_RightGrid = frc2::Button(BUTTON_L_TWO(RIGHT_GRID));

  m_NavXReset = frc2::Button(BUTTON_L(8)); //PUT Define
  
  
  m_NavXReset.WhenPressed(
    new frc2::InstantCommand([&]{
      DebugOutF("NavX Zero");
      zeroGyroscope();
  }));
  
  m_PlacingMode.WhenPressed(new frc2::ParallelCommandGroup(
		  frc2::PrintCommand("Bottom Left Placement"),
			PivotToPos(Robot::GetRobot()->GetArm().m_PivotMatrix[SelectedRow][SelectedColumn]), 
      WristToPos(Robot::GetRobot()->GetArm().m_WristMatrix[SelectedRow][SelectedColumn])
	));


  m_TL.WhenPressed(new frc2::InstantCommand([&]{
    DebugOutF("m_TL");
    SelectedRow = 0;
    SelectedColumn = 0;
    frc::Pose2d SelectedPose = 
		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));


  m_ML.WhenPressed(new frc2::InstantCommand([&]{
    DebugOutF("m_ML");
    SelectedRow = 1;
    SelectedColumn = 0;
    frc::Pose2d SelectedPose = 
		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));


  m_BL.WhenPressed(new frc2::InstantCommand([&]{
    DebugOutF("m_BL");
    SelectedRow = 2;
    SelectedColumn = 0;
    frc::Pose2d SelectedPose = 
		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));


  m_TC.WhenPressed(new frc2::InstantCommand([&]{
    DebugOutF("m_TC");
    SelectedRow = 0;
    SelectedColumn = 1;
    frc::Pose2d SelectedPose = 
		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));


  m_MC.WhenPressed(new frc2::InstantCommand([&]{
    DebugOutF("m_MC");
    SelectedRow = 1;
    SelectedColumn = 1;
    frc::Pose2d SelectedPose = 
		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));


  m_BC.WhenPressed(new frc2::InstantCommand([&]{
    DebugOutF("m_BC");
    SelectedRow = 2;
    SelectedColumn = 1;
    frc::Pose2d SelectedPose = 
		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));


  m_TR.WhenPressed(new frc2::InstantCommand([&]{
    DebugOutF("m_TR");
    SelectedRow = 0;
    SelectedColumn = 2;
    frc::Pose2d SelectedPose = 
		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));


  m_MR.WhenPressed(new frc2::InstantCommand([&]{
		DebugOutF("m_MR");
		SelectedRow = 1;
		SelectedColumn = 2;
		frc::Pose2d SelectedPose = 
		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));
  

  m_BR.WhenPressed(new frc2::InstantCommand([&]{
		DebugOutF("m_BR");
		SelectedRow = 2;
		SelectedColumn = 2;
		frc::Pose2d SelectedPose = 
		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));


	m_LeftGrid.WhenPressed(
    DebugOutF("m_LeftGrid");
		if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)) {
      Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 0;
		} else{
			Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 2;
		}
  );

  
  m_CenterGrid.WhenPressed(
    Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 1;
    new frc2::ParallelCommandGroup(
		  frc2::PrintCommand("Substation Cone Pickup"),
			PivotToPos(PIVOT_SHELF_PICKUP_ANGLE), 
      WristToPos(28)
	));


  m_RightGrid.WhenPressed(new frc2::InstantCommand([&]{
		DebugOutF("m_RightGrid");
		if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
			Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 2;
		} else{
			Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 0;
	}}));
  

  // m_LeftGrid.WhenPressed(new frc2::InstantCommand([&]{
	// 	DebugOutF("m_LeftGrid");
	// 	if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
	// 		Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 0;
	// 	} else{
	// 		Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 2;
	// 	}
	// }));

	// m_CenterGrid.WhenPressed(new frc2::InstantCommand([&]{
	// 	DebugOutF("m_CenterGrid");
	// 	Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 1;
	// }));

  // new frc2::InstantCommand([&]{
	// 	DebugOutF("m_ML");
	// 	SelectedRow = 1;
	// 	SelectedColumn = 0;
	// 	frc::Pose2d SelectedPose = 
	// 		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
	// 	Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  //   })

    //   new frc2::InstantCommand([&]{
	// 	DebugOutF("m_TL");
	// 	SelectedRow = 0;
	// 	SelectedColumn = 0; 
	// 	frc::Pose2d SelectedPose = 
	// 		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
	// 	Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
	// })

  // new frc2::InstantCommand([&]{
		// DebugOutF("m_TC");
		// SelectedRow = 0;
		// SelectedColumn = 1;
		// frc::Pose2d SelectedPose = 
		// 	Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		// Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);	
		// })

    // new frc2::InstantCommand([&]{
		// DebugOutF("m_TR");
		// SelectedRow = 0;
		// SelectedColumn = 2;
		// frc::Pose2d SelectedPose = 
		// 	Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		// Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);	
		// })
}

frc::Pose2d Robot::TransformPose(frc::Pose2d SelectedPose){
	if(Robot::GetRobot()->GetDriveTrain().m_SelectedGrid == 1){
		SelectedPose = SelectedPose +
			frc::Transform2d(
				frc::Translation2d(units::meter_t(0), units::meter_t(1.68)),
				frc::Rotation2d(units::radian_t(0))
		).Inverse();
	} else if(Robot::GetRobot()->GetDriveTrain().m_SelectedGrid == 2){
		SelectedPose = SelectedPose + 
			frc::Transform2d(
				frc::Translation2d(units::meter_t(0), units::meter_t(2 * 1.68)),
				frc::Rotation2d(units::radian_t(0))
		).Inverse();		
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
  m_COBTicks++;
  Robot::GetRobot()->GetCOB().GetTable().GetEntry("/COB/pitchAngle").SetDouble(Robot::GetRobot()->GetNavX().GetPitch() + 0.05);
  m_AutoPath = std::string(Robot::GetRobot()->GetCOB().GetTable().GetEntry("/COB/auto").GetString(""));

  // DebugOutF("PosDeg: " + std::to_string(GetArm().WristTicksToDegrees(GetArm().GetWristMotor().GetSelectedSensorPosition())));
	// DebugOutF("PosTicks: " + std::to_string(GetArm().GetWristMotor().GetSelectedSensorPosition()));
  // DebugOutF("StringDeg: " + std::to_string(GetArm().WristTicksToDegrees(GetArm().WristStringPotUnitsToTicks(GetArm().GetStringPot().GetValue())-29000.0 - GetArm().WristDegreesToTicks(45))));
  // DebugOutF("PivotDeg: " + std::to_string(GetArm().PivotTicksToDegrees(GetArm().GetPivotMotor().GetSelectedSensorPosition())));

  //DebugOutF(m_AutoPath);

  // DebugOutF("CanCoder" + std::to_string(Robot::GetRobot()->GetArm().GetPivotCANCoder().GetAbsolutePosition()));
  // DebugOutF("Arm" + std::to_string(Robot::GetRobot()->GetArm().PivotTicksToDegrees(Robot::GetRobot()->GetArm().GetPivotMotor().GetSelectedSensorPosition())));

  // DebugOutF("X: " + std::to_string(Robot::GetRobot()->GetDriveTrain().m_TransformedPose.X().value()));
  // DebugOutF("Y: " + std::to_string(Robot::GetRobot()->GetDriveTrain().m_TransformedPose.Y().value()));
  // DebugOutF("Deg: " + std::to_string(Robot::GetRobot()->GetDriveTrain().m_TransformedPose.Rotation().Degrees().value()));

  // Robot::GetCOB().GetTable().GetEntry("/COB/armValue").SetDouble(Robot::GetArm().GetPot());   
  // Robot::GetCOB().GetTable().GetEntry("/COB/armAngle").SetDouble(Robot::GetArm().PivotTicksToDeg(Robot::GetArm().GetPivot().GetSelectedSensorPosition()));                                                                                                                                   
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  GetDriveTrain().BreakMode(false);
  GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);

  
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {  
  DebugOutF("Auto init");

  frc2::CommandScheduler::GetInstance().CancelAll();
  GetNavX().ZeroYaw();
  GetNavX().SetAngleAdjustment(0);
  GetDriveTrain().BreakMode(true);
  //PathPlannerTrajectory traj;

  //Load trajectory
  // if(!COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
    // DebugOutF("Blue");
  //PathPlannerTrajectory traj = PathPlanner::loadPath(m_AutoPath, PathConstraints(4_mps, 1_mps_sq));

  PathPlannerTrajectory traj = PathPlanner::loadPath("AutoBalance", PathConstraints(4_mps, 1_mps_sq));

  // } else {
  // //   DebugOutF("Red");
  // //   PathPlannerTrajectory traj = PathPlanner::loadPath("TestBalanceRed", PathConstraints(4_mps, 1_mps_sq));
  // // }


  //PathPlannerTrajectory::transformTrajectoryForAlliance(traj, frc::DriverStation::GetAlliance());

  //frc::Pose2d startingPose = frc::Pose2d(units::meter_t(2.3), units::meter_t(1.75), frc::Rotation2d(units::degree_t(0)));
  frc::Pose2d startingPose = frc::Pose2d(traj.getInitialState().pose.Translation(), frc::Rotation2d(units::degree_t(0)));

  GetDriveTrain().GetOdometry()->ResetPosition(units::radian_t(Deg2Rad(GetAngle())), 
    wpi::array<frc::SwerveModulePosition, 4>
         (GetDriveTrain().m_FrontLeftModule.GetPosition(), GetDriveTrain().m_FrontRightModule.GetPosition(), GetDriveTrain().m_BackLeftModule.GetPosition(), GetDriveTrain().m_BackRightModule.GetPosition()), 
    startingPose);
  
  
  // DebugOutF("InitialRotation: " + std::to_string(traj.getInitialHolonomicPose().Rotation().Degrees().value()));
  // DebugOutF("InitialY: " + std::to_string(traj.asWPILibTrajectory().InitialPose().Y().value()));
  // DebugOutF("InitialX: " + std::to_string(traj.asWPILibTrajectory().InitialPose().X().value()));
  
  frc2::CommandScheduler::GetInstance().Schedule(new frc2::SequentialCommandGroup(
    frc2::ParallelRaceGroup(
      frc2::WaitCommand(2_s),
      PivotToPos(PIVOT_PLACING_MID_CONE_ANGLE), 
      frc2::FunctionalCommand(
        [&] {
          GetArm().m_BottomIntake.EnableCurrentLimit(false);
          GetArm().m_BottomIntake.Set(ControlMode::PercentOutput, -1);
        },
        [&] {},
        [&](bool e) { // onEnd
          GetArm().m_BottomIntake.Set(ControlMode::PercentOutput, 0);
        },
        [&] { // isFinished
        return false;
        }
      ),
      WristToPos(WRIST_PLACING_MID_CONE_ANGLE)
    ),

    frc2::ParallelRaceGroup(
      frc2::FunctionalCommand(
        [&] {
          GetArm().m_BottomIntake.Set(ControlMode::PercentOutput, 1);
        },
        [&] {},
        [&](bool e) { // onEnd
          GetArm().m_BottomIntake.Set(ControlMode::PercentOutput, 0);
          GetArm().m_BottomIntake.EnableCurrentLimit(true);
        },
        [&] { // isFinished
        return false;
        }
      ),
      frc2::WaitCommand(0.5_s)
    ),

    frc2::ParallelRaceGroup(
      frc2::WaitCommand(1_s),
      PivotToPos(PIVOT_TRANSIT_ANGLE), 
      WristToPos(120)
    ),

    TrajectoryCommand(traj),
    // frc2::ParallelRaceGroup(
    //   frc2::WaitCommand(1_s),
    //   PivotToPos(PIVOT_GROUND_ANGLE), 
    //   WristToPos(WRIST_GROUND_ANGLE)
    // ),

    // frc2::FunctionalCommand(
    //     [&] {
    //       GetArm().m_BottomIntake.Set(ControlMode::PercentOutput, -6);
    //     },
    //     [&] {},
    //     [&](bool e) { // onEnd
    //     },
    //     [&] { // isFinished
    //       return true;
    //     }
    // ),

    // frc2::FunctionalCommand(
    //     [&] {
    //       GetArm().m_BottomIntake.EnableCurrentLimit(true);
    //       GetArm().m_BottomIntake.Set(ControlMode::PercentOutput, 0);
    //     },
    //     [&] {},
    //     [&](bool e) { // onEnd
    //     },
    //     [&] { // isFinished
    //       return true;
    //     }
    // ),

    //frc2::ParallelDeadlineGroup(
    //TrajectoryCommand(PathPlanner::loadPath("Phase3", PathConstraints(4_mps, 1_mps_sq))),
      // PivotToPos(PIVOT_TRANSIT_ANGLE), 
      // WristToPos(120)
    //),
    //frc2::WaitCommand(0.5_s),
    AutoBalance()
  ));

  //DebugOutF(GetDriveTrain().m_EventMap.find("\"Mark 1\""));
  // (GetDriveTrain().m_EventMap.at(std::string("Mark 1")).get()->Schedule());
}

void Robot::AutonomousPeriodic() {

    // DebugOutF("X: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().X().value()));
    // DebugOutF("Y: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Y().value()));
    // DebugOutF("Deg: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Rotation().Degrees().value()));
  
}

void Robot::TeleopInit() {

  //GetNavX().ZeroYaw();
  GetNavX().SetAngleAdjustment(0);
  GetDriveTrain().BreakMode(true);
   
  // frc::Pose2d startingPose = frc::Pose2d(units::meter_t(2.54), units::meter_t(1.75), frc::Rotation2d(units::degree_t(0)));
  //   GetDriveTrain().GetOdometry()->ResetPosition(units::radian_t(Deg2Rad(GetAngle())), 
  //       wpi::array<frc::SwerveModulePosition, 4>
  //           (GetDriveTrain().m_FrontLeftModule.GetPosition(), GetDriveTrain().m_FrontRightModule.GetPosition(), GetDriveTrain().m_BackLeftModule.GetPosition(), GetDriveTrain().m_BackRightModule.GetPosition()), 
  //       startingPose);

  // m_Arm.PlaceElement(0,0);
}

/**
 * This function is called periodically during operator control.  
 */
void Robot::TeleopPeriodic() {
  // frc2::CommandScheduler::GetInstance().Run();
  // frc2::CommandScheduler::GetInstance().Schedule(m_Arm.Telescope(50));  

  
  // DebugOutF("LLX: " + std::to_string(m_Vision.GetPoseBlue().X().value()));
  // DebugOutF("LLY: " + std::to_string(m_Vision.GetPoseBlue().Y().value()));
  // DebugOutF("LLZ: " + std::to_string(m_Vision.GetPoseBlue().Rotation().Degrees().value()));

  DebugOutF("BL: " + std::to_string(Rad2Deg(GetDriveTrain().m_BackLeftModule.GetSteerAngle())));
  DebugOutF("BR: " + std::to_string(Rad2Deg(GetDriveTrain().m_BackRightModule.GetSteerAngle())));
  DebugOutF("FL: " + std::to_string(Rad2Deg(GetDriveTrain().m_FrontLeftModule.GetSteerAngle())));
  DebugOutF("FR: " + std::to_string(Rad2Deg(GetDriveTrain().m_FrontRightModule.GetSteerAngle())));
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
