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
			 m_Extraction(EXTRACTION_MOTOR),
			 m_Intake(6),

			 //BUTTONBOARD 1
			 m_Override(BUTTON_L(ARM_OVERRIDE)),
			 m_Override2(BUTTON_L(2)),

			 m_ConeMode(BUTTON_L(CONE_MODE)),
			 m_CubeMode(BUTTON_L(CUBE_MODE)),

			 m_FrontMode(BUTTON_L(FRONT_MODE)),
			 m_BackMode(BUTTON_L(BACK_MODE)),

			 m_ManualArmBrake(BUTTON_L(MANUAL_ARM_BRAKE)),
			 m_ManualSlipBrake(BUTTON_L(MANUAL_SLIP_BRAKE)),

			m_Squeeze(BUTTON_L_TWO(6)),

			 m_TransitMode(BUTTON_L_TWO(TRANSIT_MODE)),
			 m_GroundPickupMode(BUTTON_L_TWO(GROUND_PICKUP_MODE)),
			 m_LoadingMode(BUTTON_L_TWO(LOADING_MODE)),

			m_Timer()
			{}

void Arm::Init()
{
	shouldSqueeze = true;
	m_Pivot.SetSelectedSensorPosition(PivotDegToTicks(OFFSET_FROM_VERTICAL));

	SetButtons();
	m_Pivot.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	m_Extraction.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

	// set PID values
	m_Pivot.ConfigAllowableClosedloopError(0, PIVOT_ERROR);
	m_Pivot.Config_kP(0, PIVOT_KP);
	m_Pivot.Config_kI(0, PIVOT_KI);
	m_Pivot.Config_kD(0, PIVOT_KD);
	m_Extraction.ConfigAllowableClosedloopError(0, EXTRACTION_ERROR);
	m_Extraction.Config_kP(0, EXTRACTION_KP);
	m_Extraction.Config_kI(0, EXTRACTION_KI);
	m_Extraction.Config_kD(0, EXTRACTION_KD);
}

void Arm::SetButtons()
{

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
	return new frc2::FunctionalCommand([&] {},
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