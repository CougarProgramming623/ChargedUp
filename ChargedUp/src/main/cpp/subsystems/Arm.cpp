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


Arm::Arm() : m_Pivot(PIVOT_MOTOR),
			 m_Wrist(WRIST_MOTOR),
			 m_Intake(INTAKE_MOTOR),

			 //BUTTONBOARD 1
			 m_Override(BUTTON_L(ARM_OVERRIDE)),
			 m_Override2(BUTTON_L(2)),

			 m_ConeMode(BUTTON_L(CONE_MODE)),
			 m_CubeMode(BUTTON_L(CUBE_MODE)),

			 m_TransitMode(BUTTON_L_TWO(TRANSIT_MODE)),
			 m_GroundPickupMode(BUTTON_L_TWO(GROUND_PICKUP_MODE)),
			 m_LoadingMode(BUTTON_L_TWO(LOADING_MODE)),

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
	m_Intake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

	// set PID values
	m_Pivot.ConfigAllowableClosedloopError(0, PIVOT_ERROR);
	m_Pivot.Config_kP(0, PIVOT_KP);
	m_Pivot.Config_kI(0, PIVOT_KI);
	m_Pivot.Config_kD(0, PIVOT_KD);
	
}

void Arm::SetButtons()
{
	//m_Override.WhenPressed(frc2::InstantCommand([&] {frc2::CommandScheduler::GetInstance().CancelAll()};));
	// m_Override2.WhenPressed(ManualControls());

	// m_ManualArmBrake.WhenPressed(ManualArmBrake());
	// m_ManualSlipBrake.WhenPressed(ManualSlipBrake());

	
	// m_TR.WhenPressed(m_Top);

	// m_TC.WhenPressed(m_Mid);

	// m_GroundPickupMode.WhenPressed(m_Bot);
	// m_TransitMode.WhenPressed(TransitMode());
	// m_LoadingMode.WhenPressed(LoadingMode());

	Robot::GetRobot()->m_TL.WhenPressed(frc2::InstantCommand([&]{
		DebugOutF("TL");
		m_Intake.EnableCurrentLimit(false);
		m_Intake.EnableCurrentLimit(true);
		m_Intake.Set(ControlMode::PercentOutput, .55);
	}));

	Robot::GetRobot()->m_TC.WhenPressed(frc2::InstantCommand([&]{
		DebugOutF("TC");
		m_Intake.EnableCurrentLimit(false);
		m_Intake.EnableCurrentLimit(true);
		m_Intake.Set(ControlMode::PercentOutput, 0);
	}));

	Robot::GetRobot()->m_TR.WhenPressed(frc2::InstantCommand([&]{
		DebugOutF("TR");
		m_Intake.EnableCurrentLimit(false);
		m_Intake.EnableCurrentLimit(true);

		m_Intake.Set(ControlMode::PercentOutput, -.55);
	}));

	m_Intake.ConfigPeakCurrentDuration(1750);
	m_Intake.ConfigPeakCurrentLimit(6);
	m_Intake.ConfigContinuousCurrentLimit(2);
	m_Intake.EnableCurrentLimit(true);

}

// while override is active, gives manual joysticks control over the two arm motors
frc2::FunctionalCommand* Arm::ManualControls()
{
	return new frc2::FunctionalCommand([&] { // onInit
		//empty
	},
	[&] { // onExecute
		m_Pivot.Set(ControlMode::PercentOutput, Robot::GetRobot()->GetJoystick().GetRawAxis(PIVOT_CONTROL) / 5);
	},
	[&](bool e) { // onEnd
		m_Pivot.Set(ControlMode::PercentOutput, 0);
	},
	[&] { // isFinished
		return !Robot::GetRobot()->GetButtonBoard().GetRawButton(ARM_OVERRIDE);
	});
}