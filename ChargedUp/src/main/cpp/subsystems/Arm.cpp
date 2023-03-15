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
			 m_Override2(BUTTON_L(ARM_OVERRIDE_2)),

			 m_ConeMode(BUTTON_L(CONE_MODE)),
			 m_CubeMode(BUTTON_L(CUBE_MODE)),

			 m_IntakeButton(BUTTON_L(INTAKE_BUTTON)),
			 m_OuttakeButton(BUTTON_L(OUTTAKE_BUTTON)),

			 m_TransitMode(BUTTON_L_TWO(TRANSIT_MODE)),
			 m_GroundPickupMode(BUTTON_L_TWO(GROUND_PICKUP_MODE)),
			 m_PlacingMode(BUTTON_L_TWO(PLACING_MODE)),

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

	m_Pivot.ConfigAllowableClosedloopError(0, PIVOT_ERROR);
	m_Pivot.Config_kP(0, PIVOT_KP);
	m_Pivot.Config_kI(0, PIVOT_KI);
	m_Pivot.Config_kD(0, PIVOT_KD);

	m_Intake.ConfigPeakCurrentDuration(1750);
	m_Intake.ConfigPeakCurrentLimit(6);
	m_Intake.ConfigContinuousCurrentLimit(2);
	m_Intake.EnableCurrentLimit(true);
}

void Arm::SetButtons()
{
	//m_Override.WhenPressed(frc2::InstantCommand([&] {frc2::CommandScheduler::GetInstance().CancelAll()};));
	m_Override2.WhenPressed(ManualControls());

	m_IntakeButton.WhenPressed(DynamicIntake());
	m_OuttakeButton.WhenPressed(DynamicIntake());
}

// while override is active, gives manual joysticks control over the two arm motors
frc2::FunctionalCommand* Arm::ManualControls()
{
	return new frc2::FunctionalCommand([&] { // onInit
		//empty
	},
	[&] { // onExecute
		m_Pivot.Set(ControlMode::PercentOutput, Robot::GetRobot()->GetJoystick().GetRawAxis(PIVOT_CONTROL) / 5);
		m_Wrist.Set(ControlMode::PercentOutput, Robot::GetRobot()->GetJoystick().GetRawAxis(WRIST_CONTROL) / 5);
		PrintPot();
	},
	[&](bool e) { // onEnd
		m_Pivot.Set(ControlMode::PercentOutput, 0);
		m_Wrist.Set(ControlMode::PercentOutput, 0);
	},
	[&] { // isFinished
		return !Robot::GetRobot()->GetButtonBoard().GetRawButton(ARM_OVERRIDE);
	});
}