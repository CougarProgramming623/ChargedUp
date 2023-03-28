#include "subsystems/Arm.h"
#include "Robot.h"
#include "frc2/command/PrintCommand.h"
#include <frc/DriverStation.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include "./commands/DriveToPosCommand.h"

using ctre::phoenix::motorcontrol::ControlMode;
using ctre::phoenix::motorcontrol::can::TalonFX;
using ctre::phoenix::motorcontrol::can::TalonSRX;
using ctre::phoenix::sensors::AbsoluteSensorRange;


Arm::Arm() : m_Pivot(PIVOT_MOTOR),
			 m_Wrist(WRIST_MOTOR),
			//  m_TopIntake(TOP_INTAKE_MOTOR),
			 m_BottomIntake(BOTTOM_INTAKE_MOTOR),

			 //BUTTONBOARD 1
			 m_Override(BUTTON_L(ARM_OVERRIDE)),
			 m_Override2(BUTTON_L(ARM_OVERRIDE_2)),

			 m_ConeMode(BUTTON_L(CONE_MODE)),
			 m_CubeMode(BUTTON_L(CUBE_MODE)),

			 m_IntakeButton(BUTTON_L(INTAKE_BUTTON)),
			 m_OuttakeButton(BUTTON_L(OUTTAKE_BUTTON)),

			 

			m_Timer()


			// m_Top(PlaceElement(0, 2)),

			// m_Mid(PlaceElement(1, 2)),

			// m_Bot(PlaceElement(2, 2))
			{}

void Arm::Init()
{
	SetButtons();

	m_Pivot.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	m_Wrist.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	// m_TopIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	m_BottomIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

	m_Pivot.ConfigAllowableClosedloopError(0, 10);
	m_Pivot.Config_kP(0, 0.01);
	m_Pivot.Config_kI(0, 0.00001); //0.000005
	m_Pivot.Config_kD(0, 0.3);
	m_Pivot.Config_kF(0, 0.0639375, 0);
	m_Pivot.ConfigMotionCruiseVelocity(8000, 0); //8400 working value
	m_Pivot.ConfigMotionAcceleration(8000, 0); //16800 working value

	m_Wrist.ConfigAllowableClosedloopError(0, 0);
	m_Wrist.Config_kP(0, 0.0175); //0.009
	m_Wrist.Config_kI(0, 0.000002);
	m_Wrist.Config_kD(0, 0.6);
	m_Wrist.Config_kF(0, 0.06089285714, 0);
	m_Wrist.ConfigMotionCruiseVelocity(8400, 0); //8400 working value
	m_Wrist.ConfigMotionAcceleration(16800, 0); //16800 working value

	

	// m_TopIntake.ConfigPeakCurrentDuration(1750);
	// m_TopIntake.ConfigPeakCurrentLimit(6);
	// m_TopIntake.ConfigContinuousCurrentLimit(2);
	// m_TopIntake.EnableCurrentLimit(true);
	m_BottomIntake.ConfigPeakCurrentDuration(1750);
	m_BottomIntake.ConfigPeakCurrentLimit(7);
	m_BottomIntake.ConfigContinuousCurrentLimit(3.5);
	m_BottomIntake.EnableCurrentLimit(true);
 

	m_PivotCANCoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Unsigned_0_to_360);
	m_Pivot.SetSelectedSensorPosition((CANCODER_ZERO - m_PivotCANCoder.GetAbsolutePosition()) * PIVOT_TICKS_PER_DEGREE);
	m_Wrist.SetSelectedSensorPosition((WristStringPotUnitsToTicks(m_StringPot.GetValue()))-29000.0 - WristDegreesToTicks(45));
}

void Arm::SetButtons()
{
	m_Override.WhenPressed(ManualControls());

	m_IntakeButton.WhenPressed(DynamicIntake());
	m_OuttakeButton.WhenPressed(DynamicIntake());


	// m_GroundPickupMode.WhenPressed(
	// 	new frc2::ParallelCommandGroup(
	// 		frc2::PrintCommand("-45"),
	// 		PivotToPos(PIVOT_GROUND_ANGLE)
	// 	)
	// 	// new WristToPos(WRIST_GROUND_ANGLE)
	// );

	// m_TransitMode.WhenPressed(
	// 	new frc2::ParallelCommandGroup(
	// 		frc2::PrintCommand("0"),
	// 		PivotToPos(0)
	// 	)		// new WristToPos(WRIST_TRANSIT_ANGLE)
	// );

	// m_PlacingMode.WhenPressed(
	// 	new frc2::ParallelCommandGroup(
	// 		frc2::PrintCommand("45"),
	// 		PivotToPos(45)
	// 	)		// new WristToPos(WRIST_PLACING_MID_CUBE_ANGLE)
	// );

	

	
}

// while override is active, gives manual joysticks control over the two arm motors
frc2::FunctionalCommand* Arm::ManualControls()
{
	return new frc2::FunctionalCommand([&] { // onInit
		//empty
	},
	[&] { // onExecute
		m_Pivot.Set(ControlMode::PercentOutput, Robot::GetRobot()->GetButtonBoard().GetRawAxis(PIVOT_CONTROL) / 2);
		m_Wrist.Set(ControlMode::PercentOutput, Robot::GetRobot()->GetButtonBoard().GetRawAxis(WRIST_CONTROL) / 2);

	// ---------------------------------------------------------------------------------------

	//  double power = .55;
	// 	if(Robot::GetRobot()->GetButtonBoard().GetRawButton(CUBE_MODE)) {
	// 		if(Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_BUTTON)) {
	// 			m_TopIntake.Set(ControlMode::PercentOutput, power);
	// 			m_BottomIntake.Set(ControlMode::PercentOutput, power);
	// 		} else if (Robot::GetRobot()->GetButtonBoard().GetRawButton(OUTTAKE_BUTTON)) {
	// 			m_TopIntake.Set(ControlMode::PercentOutput, -power);
	// 			m_BottomIntake.Set(ControlMode::PercentOutput, -power);
	// 		}
	// 	} else if (Robot::GetRobot()->GetButtonBoard().GetRawButton(CONE_MODE)) {
	// 		if(Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_BUTTON)) {f
	// 			m_TopIntake.Set(ControlMode::PercentOutput, power);
	// 			m_BottomIntake.Set(ControlMode::PercentOutput, -power);
	// 		} else if (Robot::GetRobot()->GetButtonBoard().GetRawButton(OUTTAKE_BUTTON)) {
	// 			m_TopIntake.Set(ControlMode::PercentOutput, -power);
	// 			m_BottomIntake.Set(ControlMode::PercentOutput, power);
	// 		}
	// }

	//---------------------------------------------------------------------------------------------
	
	double power = -.7; //default power for cone
	
	if(Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_BUTTON)) m_BottomIntake.Set(ControlMode::PercentOutput, power);
	else if (Robot::GetRobot()->GetButtonBoard().GetRawButton(OUTTAKE_BUTTON)) m_BottomIntake.Set(ControlMode::PercentOutput, 1);
	else m_BottomIntake.Set(ControlMode::PercentOutput, 0);

	},[&](bool e) { // onEnd
		m_Pivot.Set(ControlMode::PercentOutput, 0);
		m_Wrist.Set(ControlMode::PercentOutput, 0);
		m_BottomIntake.Set(ControlMode::PercentOutput, 0);
		// m_BottomIntake.Set(ControlMode::PercentOutput, 0);
	},
	[&] { // isFinished
		return !Robot::GetRobot()->GetButtonBoard().GetRawButton(ARM_OVERRIDE);
	});
}