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
  m_Arm.Init();
  AutoButtons();
  m_COBTicks = 0;
  m_AutoPath = "";
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
  m_SingleSubCube = frc2::Button(BUTTON_L_TWO(13));
  m_DoubleSub   = frc2::Button(BUTTON_L(7));

  m_LeftGrid    = frc2::Button(BUTTON_L_TWO(LEFT_GRID));
  m_CenterGrid  = frc2::Button(BUTTON_L_TWO(CENTER_GRID));
  m_RightGrid   = frc2::Button(BUTTON_L_TWO(RIGHT_GRID));

  // m_MidCone = frc2::Button(BUTTON_L_TWO(TRANSIT_MODE));
	// m_MidCube = frc2::Button(BUTTON_L_TWO(GROUND_PICKUP_MODE));
  m_PlacingMode = frc2::Button(BUTTON_L_TWO(PLACING_MODE));
  m_GroundPickup = frc2::Button(BUTTON_L_TWO(GROUND_PICKUP_MODE));

  m_NavXReset = frc2::Button(BUTTON_L(8)); //PUT Define
  GetArm().m_PlacingMode = frc2::Button(BUTTON_L_TWO(15));
  m_AutoBalance = frc2::Button(BUTTON_L(3));
   m_Print = frc2::Button(BUTTON_L(2));

  m_AutoBalance.WhileHeld(new AutoBalance());
 
  
  m_Print.WhileHeld(
    new frc2::InstantCommand([&]{
      DebugOutF("StringDeg: " + std::to_string(GetArm().WristTicksToDegrees(GetArm().WristStringPotUnitsToTicks(GetArm().GetStringPot().GetValue()))));
      DebugOutF("PivotDeg: " + std::to_string(GetArm().PivotTicksToDegrees(GetArm().GetPivotMotor().GetSelectedSensorPosition())));
      DebugOutF("StringPotRaw: " + std::to_string(GetArm().GetStringPot().GetValue()));
    })
  );

  
  m_NavXReset.WhenPressed(
    new frc2::InstantCommand([&]{
      DebugOutF("NavX Zero");
      zeroGyroscope();
  }));
  
  // GetArm().m_PlacingMode.WhenPressed(new frc2::ParallelCommandGroup(
	// 		PivotToPos(Robot::GetRobot()->GetArm().m_PivotMatrix[SelectedRow][SelectedColumn]), 
  //     WristToPos(Robot::GetRobot()->GetArm().m_WristMatrix[SelectedRow][SelectedColumn])
	// ));

  m_GroundPickup.WhenPressed(
    //DebugOutF("Ground Pickup");
    new frc2::ParallelCommandGroup(
        frc2::InstantCommand([&]{Robot::GetRobot()->GetArm().SetMotionMagicValues(PIVOT_DFLT_VEL, PIVOT_DFLT_ACC, WRIST_DFLT_VEL, WRIST_DFLT_ACC);}),
        PivotToPos(94.0), 
        WristToPos(-2.0)
      )
  );

  m_SingleSub.WhenPressed(
    //DebugOutF("Single Substation");
    new frc2::ParallelCommandGroup(
        frc2::InstantCommand([&]{Robot::GetRobot()->GetArm().SetMotionMagicValues(PIVOT_DFLT_VEL, PIVOT_DFLT_ACC, WRIST_DFLT_VEL, WRIST_DFLT_ACC);}),
        PivotToPos(Robot::GetRobot()->GetArm().m_PivotMatrix[0][2]), 
        WristToPos(Robot::GetRobot()->GetArm().m_WristMatrix[0][2])
      )
  );

  m_SingleSubCube.WhenPressed(
    //DebugOutF("Single Substation");
    new frc2::ParallelCommandGroup(
        frc2::InstantCommand([&]{Robot::GetRobot()->GetArm().SetMotionMagicValues(PIVOT_DFLT_VEL, PIVOT_DFLT_ACC, WRIST_DFLT_VEL, WRIST_DFLT_ACC);}),
        PivotToPos(72.0), 
        WristToPos(54.0)
      )
  );

  m_DoubleSub.WhenPressed(
    //DebugOutF("Double Substation");
    new frc2::ParallelCommandGroup(
      frc2::InstantCommand([&]{Robot::GetRobot()->GetArm().SetMotionMagicValues(PIVOT_DFLT_VEL / 2, PIVOT_DFLT_ACC / 4, WRIST_DFLT_VEL, WRIST_DFLT_ACC);}),
      PivotToPos(Robot::GetRobot()->GetArm().m_PivotMatrix[0][0]), 
      WristToPos(Robot::GetRobot()->GetArm().m_WristMatrix[0][0])
    )
  );


  m_TL.WhenPressed(new frc2::InstantCommand([&] {
    DebugOutF("m_TL - nothing");
  }));


  m_ML.WhenPressed(new frc2::InstantCommand([&]{
    DebugOutF("m_ML");

    GetArm().m_PlacingMode.WhenPressed(frc2::ParallelCommandGroup(
        frc2::InstantCommand([&]{Robot::GetRobot()->GetArm().SetMotionMagicValues(PIVOT_DFLT_VEL / 2, PIVOT_DFLT_ACC / 4, WRIST_DFLT_VEL, WRIST_DFLT_ACC);}),
        PivotToPos(Robot::GetRobot()->GetArm().m_PivotMatrix[1][0]), 
        WristToPos(Robot::GetRobot()->GetArm().m_WristMatrix[1][0])
      ));

    frc::Pose2d SelectedPose = 
		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));


  m_BL.WhenPressed(new frc2::InstantCommand([&]{
    DebugOutF("m_BL");
    GetArm().m_PlacingMode.WhenPressed(frc2::ParallelCommandGroup(
        PivotToPos(Robot::GetRobot()->GetArm().m_PivotMatrix[2][0]), 
        WristToPos(Robot::GetRobot()->GetArm().m_WristMatrix[2][0])
      ));
    frc::Pose2d SelectedPose = 
		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));


  m_TC.WhenPressed(new frc2::InstantCommand([&]{
    DebugOutF("m_TC");
    GetArm().m_PlacingMode.WhenPressed(frc2::ParallelCommandGroup(
        frc2::InstantCommand([&]{Robot::GetRobot()->GetArm().SetMotionMagicValues(PIVOT_DFLT_VEL / 2, PIVOT_DFLT_ACC / 4, WRIST_DFLT_VEL, WRIST_DFLT_ACC);}),
        PivotToPos(Robot::GetRobot()->GetArm().m_PivotMatrix[0][1]), 
        WristToPos(Robot::GetRobot()->GetArm().m_WristMatrix[0][1])
      ));
    frc::Pose2d SelectedPose = 
		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));


  m_MC.WhenPressed(new frc2::InstantCommand([&]{
    DebugOutF("m_MC");
    GetArm().m_PlacingMode.WhenPressed(frc2::ParallelCommandGroup(
        PivotToPos(Robot::GetRobot()->GetArm().m_PivotMatrix[1][1]), 
        WristToPos(Robot::GetRobot()->GetArm().m_WristMatrix[1][1])
      ));
    frc::Pose2d SelectedPose = 
		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));

  m_BC.WhenPressed(new frc2::InstantCommand([&]{
    DebugOutF("m_BC");
    GetArm().m_PlacingMode.WhenPressed(frc2::ParallelCommandGroup(
        PivotToPos(Robot::GetRobot()->GetArm().m_PivotMatrix[2][1]), 
        WristToPos(Robot::GetRobot()->GetArm().m_WristMatrix[2][1])
      ));
    frc::Pose2d SelectedPose = 
		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));


  m_TR.WhenPressed(new frc2::InstantCommand([&]{
    DebugOutF("m_TR - nothing");
    // frc::Pose2d SelectedPose = 
		// Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		// Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));

  m_MR.WhenPressed(new frc2::InstantCommand([&]{
		DebugOutF("m_MR");
		GetArm().m_PlacingMode.WhenPressed(frc2::ParallelCommandGroup(
        frc2::InstantCommand([&]{Robot::GetRobot()->GetArm().SetMotionMagicValues(PIVOT_DFLT_VEL / 2, PIVOT_DFLT_ACC / 4, WRIST_DFLT_VEL, WRIST_DFLT_ACC);}),
        PivotToPos(Robot::GetRobot()->GetArm().m_PivotMatrix[1][2]), 
        WristToPos(Robot::GetRobot()->GetArm().m_WristMatrix[1][2])
      ));
		frc::Pose2d SelectedPose = 
		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));

  m_BR.WhenPressed(new frc2::InstantCommand([&]{
		DebugOutF("m_BR");
		GetArm().m_PlacingMode.WhenPressed(frc2::ParallelCommandGroup(
        PivotToPos(Robot::GetRobot()->GetArm().m_PivotMatrix[2][2]), 
        WristToPos(Robot::GetRobot()->GetArm().m_WristMatrix[2][2])
      ));
		frc::Pose2d SelectedPose = 
		Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
  }));

  m_BigRed.WhenPressed(frc2::ParallelCommandGroup(
        frc2::InstantCommand([&]{Robot::GetRobot()->GetArm().SetMotionMagicValues(PIVOT_DFLT_VEL, PIVOT_DFLT_ACC, WRIST_DFLT_VEL, WRIST_DFLT_ACC);}),
        frc2::SequentialCommandGroup(
          frc2::WaitCommand(0.25_s),
          PivotToPos(92.0)
        ),      
        WristToPos(127.0)
      )
	);

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
  // DebugOutF("Row: " + std::to_string(SelectedRow) + " , Col: " + std::to_string(SelectedColumn));


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
      PivotToPos(-22.0), 
      frc2::FunctionalCommand(
        [&] {
          GetArm().GetBottomIntakeMotor().EnableCurrentLimit(false);
          GetArm().GetBottomIntakeMotor().Set(ControlMode::PercentOutput, -1);
        },
        [&] {},
        [&](bool e) { // onEnd
          GetArm().GetBottomIntakeMotor().Set(ControlMode::PercentOutput, 0);
        },
        [&] { // isFinished
        return false;
        }
      ),
      WristToPos(28.0)
    ),

    frc2::ParallelRaceGroup(
      frc2::FunctionalCommand(
        [&] {
          GetArm().GetBottomIntakeMotor().Set(ControlMode::PercentOutput, 1);
        },
        [&] {},
        [&](bool e) { // onEnd
          GetArm().GetBottomIntakeMotor().Set(ControlMode::PercentOutput, 0);
          GetArm().GetBottomIntakeMotor().EnableCurrentLimit(true);
        },
        [&] { // isFinished
        return false;
        }
      ),
      frc2::WaitCommand(0.5_s)
    ),

    frc2::ParallelRaceGroup(
      frc2::WaitCommand(1_s),
      PivotToPos(92.0), 
      WristToPos(120)
    ),

    TrajectoryCommand(traj),
    // frc2::ParallelRaceGroup(
    //   frc2::WaitCommand(1_s),
    //   PivotToPos(98.0), 
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
      // PivotToPos(98.0), 
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
  // m_MMT.MotionMagicTestInit();

  //GetNavX().ZeroYaw();
  GetNavX().SetAngleAdjustment(0);
  // GetDriveTrain().BreakMode(true);
   
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

  // DebugOutF("BL: " + std::to_string(Rad2Deg(GetDriveTrain().m_BackLeftModule.GetSteerAngle())));
  // DebugOutF("BR: " + std::to_string(Rad2Deg(GetDriveTrain().m_BackRightModule.GetSteerAngle())));
  // DebugOutF("FL: " + std::to_string(Rad2Deg(GetDriveTrain().m_FrontLeftModule.GetSteerAngle())));
  // DebugOutF("FR: " + std::to_string(Rad2Deg(GetDriveTrain().m_FrontRightModule.GetSteerAngle())));
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
