#include "subsystems/Arm.h"
#include "Robot.h"
#include "frc2/command/PrintCommand.h"

using ctre::phoenix::motorcontrol::ControlMode;
using ctre::phoenix::motorcontrol::can::TalonFX;

Arm::Arm() : m_Pivot(PIVOT_MOTOR),
			 m_Extraction(EXTRACTION_MOTOR),
			 m_LeftBrake(LEFT_BRAKE),
			 m_RightBrake(RIGHT_BRAKE),
			 m_SlipBrake(SLIP_BRAKE),

			 //BUTTONBOARD 1
			 m_Override(BUTTON_L(ARM_OVERRIDE)),

			 m_ConeMode(BUTTON_L(CONE_MODE)),
			 m_CubeMode(BUTTON_L(CUBE_MODE)),

			 m_FrontMode(BUTTON_L(FRONT_MODE)),
			 m_BackMode(BUTTON_L(BACK_MODE)),

			 m_ManualArmBrake(BUTTON_L(MANUAL_ARM_BRAKE)),
			 m_ManualSlipBrake(BUTTON_L(MANUAL_SLIP_BRAKE)),

			//BUTTONBOARD 2
			 m_TL(BUTTON_L_TWO(GRID_TL)),
			 m_TC(BUTTON_L_TWO(GRID_TC)),
			 m_TR(BUTTON_L_TWO(GRID_TR)),
			 m_ML(BUTTON_L_TWO(GRID_ML)),
			 m_MC(BUTTON_L_TWO(GRID_MC)),
			 m_MR(BUTTON_L_TWO(GRID_MR)),
			 m_BL(BUTTON_L_TWO(GRID_BL)),
			 m_BC(BUTTON_L_TWO(GRID_BC)),
			 m_BR(BUTTON_L_TWO(GRID_BR)),

			 m_LeftGrid(BUTTON_L_TWO(LEFT_GRID)),
			 m_CenterGrid(BUTTON_L_TWO(CENTER_GRID)),
			 m_RightGrid(BUTTON_L_TWO(RIGHT_GRID)),

			 m_TransitMode(BUTTON_L_TWO(TRANSIT_MODE)),
			 m_GroundPickupMode(BUTTON_L_TWO(GROUND_PICKUP_MODE)),
			 m_LoadingMode(BUTTON_L_TWO(LOADING_MODE))
			{}

void Arm::Init()
{
	// m_Pivot.SetSelectedSensorPosition(0);
	// ArmBrakes(true);
	// SlipBrakes(true);

	SetButtons();
	// Brake(false);
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
	// m_Override.WhenPressed(frc2::SequentialCommandGroup(
	// 	frc2::InstantCommand([&]
	// 						 { frc2::CommandScheduler::GetInstance().CancelAll(); }),
	// 	ManualControls()));

	// m_TL.WhenPressed(PlaceElement(CONE, 0, 0));
	// m_TC.WhenPressed(PlaceElement(CUBE, 0, 1));
	// m_TR.WhenPressed(PlaceElement(CONE, 0, 2));
	// m_ML.WhenPressed(PlaceElement(CONE, 1, 0));
	// m_MC.WhenPressed(PlaceElement(CUBE, 1, 1));
	// m_MR.WhenPressed(PlaceElement(CONE, 1, 2));
	// m_BL.WhenPressed(PlaceElement(CONE, 2, 0));
	// m_BC.WhenPressed(PlaceElement(CUBE, 2, 1));
	// m_BR.WhenPressed(PlaceElement(CONE, 2, 2));

	// // m_LeftGrid.WhenPressed(/*drive to Left grid*/);
	// // m_MiddleGrid.WhenPressed(/*drive to Middle grid*/);
	// // m_RightGrid.WhenPressed(/*drive to Right grid*/);

	// m_FrontMode.WhenPressed(frc2::InstantCommand([&]
	// 											 { isOnFrontSide = true; }));
	// m_BackMode.WhenPressed(frc2::InstantCommand([&]
	// 											{ isOnFrontSide = false; }));

	// m_TransitMode.WhenPressed(TransitMode());
	// m_GroundPickupMode.WhenPressed(GroundPickupMode());
	// m_LoadingMode.WhenPressed(LoadingMode());

	// m_ManualArmBrake.WhenPressed(frc2::InstantCommand([&] {
	// 	if (m_RightBrake.Get() == 0) ArmBrakes(false);
	// 	else ArmBrakes(true);
	// }));

	// m_ManualSlipBrake.WhenPressed(frc2::InstantCommand([&] {
	// 	if (m_SlipBrake.Get() != 1) SlipBrakes(false);
	// 	else SlipBrakes(true);
	// }));

	m_TL.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_TL");}));
	m_TC.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_TC");}));
	m_TR.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_TR");}));
	m_ML.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_ML");}));
	m_MC.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_MC");}));
	m_MR.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_MR");}));
	m_BL.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_BL");}));
	m_BC.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_BC");}));
	m_BR.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_BR");}));

	m_LeftGrid.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_LeftGrid");}));
	m_CenterGrid.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_CenterGrid");}));
	m_RightGrid.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_RightGrid");}));

	m_TransitMode.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_TransitMode");}));
	m_GroundPickupMode.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_GroundPickupMode");}));
	m_LoadingMode.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_LoadingMode");}));

	m_Override.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_Override");}));

	m_ConeMode.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_ConeMode");}));
	m_CubeMode.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_CubeMode");}));

	m_FrontMode.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_FrontMode");}));
	m_BackMode.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_BackMode");}));
	
	m_ManualArmBrake.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_ManualArmBrakes");}));
	m_ManualSlipBrake.WhenPressed(frc2::InstantCommand([&]{DebugOutF("m_ManualSlipBrake");}));

}

void Arm::ArmBrakes(bool shouldBrake)
{
	if (shouldBrake)
	{
		m_LeftBrake.Set(0);
		m_RightBrake.Set(0);
	}
	else
	{
		m_LeftBrake.Set(1);
		m_RightBrake.Set(1);
	}
}

void Arm::SlipBrakes(bool shouldBrake)
{
	if (shouldBrake)
	{
		m_SlipBrake.Set(.55);
	}
	else
	{
		m_SlipBrake.Set(0);
	}
}

void Arm::PivotToPosition(double angleSetpoint)
{
	StartingTicks = m_Pivot.GetSelectedSensorPosition();
	double currentAngle = PivotTicksToDeg(StartingTicks); // current angle of the arm
	double degToMove = angleSetpoint - currentAngle;	  // how many degrees the arm needs to move in the correct direction
	TicksToMove = PivotDegToTicks(degToMove);			  // how many ticks the pivot motor needs to move in the correct direction
	Setpoint = StartingTicks + TicksToMove;

	m_Pivot.Set(ControlMode::Position, Setpoint);
}

// length should be a setpoint in inches
frc2::FunctionalCommand* Arm::Telescope(double Setpoint) {
	SetpointLength = Setpoint;
	return new frc2::FunctionalCommand(
		[&] { // onInit
			ArmBrakes(false);
			SlipBrakes(false);
		},
		[&] { // onExecute
			ArmLength = StringPotUnitsToInches(m_StringPot.GetValue()) + 1 + ARM_MINIMUM_LENGTH;
			if(ArmLength < SetpointLength) m_Extraction.Set(ControlMode::PercentOutput, -.2);
			else if(ArmLength > SetpointLength) m_Extraction.Set(ControlMode::PercentOutput, .2);
		},
		[&](bool e) { // onEnd
			m_Extraction.Set(ControlMode::PercentOutput, 0);
			ArmBrakes(true);
			SlipBrakes(true);
		},
		[&] { // isFinished
			return (abs(ArmLength - SetpointLength) < 3);
		});
}

frc2::FunctionalCommand Arm::Squeeze(bool shouldSqueeze)
{
	if (shouldSqueeze)
	{
		return frc2::FunctionalCommand([&] { // onInit
			TicksToUndoSqueeze = m_Extraction.GetSelectedSensorPosition();
			SlipBrakes(false);
			ArmBrakes(true);
			m_Extraction.Set(ControlMode::PercentOutput, .1);
		},
		[&] { // onExecute
			// empty
		},
		[&](bool e) { // onEnd
			m_Extraction.Set(ControlMode::PercentOutput, EXTRACTION_MOTOR_HOLD_POWER);
			SlipBrakes(true);
			TicksToUndoSqueeze = TicksToUndoSqueeze - m_Extraction.GetSelectedSensorPosition();
		},
		[&] { // isFinished
			return m_Extraction.GetSupplyCurrent() > SQUEEZE_AMP_THRESHOLD;
		});
	}
	else
	{
		return frc2::FunctionalCommand([&] { // onInit
			SlipBrakes(false);
			ArmBrakes(false);
		},[&] { // onExecute
			m_Extraction.Set(ControlMode::Position, m_Extraction.GetSelectedSensorPosition() - TicksToUndoSqueeze);
		},[&](bool e) { // onEnd
			m_Extraction.Set(ControlMode::PercentOutput, 0);
			SlipBrakes(true);
			ArmBrakes(false);
		},
		[&] { // isFinished
			return m_Extraction.GetSelectedSensorPosition() - TicksToUndoSqueeze < 100;
		});
	}
}

// sets arm to a set angle and radius based on element being places
// type can be either CONE or CUBE
frc2::InstantCommand Arm::PlaceElement(int type, int row, int column)
{
	return frc2::InstantCommand([&]
								{
		//index 0 is TL; index 8 is BR (read like a book)
		double FrontRadiiValues[9] = {
			FRONT_HIGH_CONE_RADIUS,   FRONT_HIGH_CUBE_RADIUS,  FRONT_HIGH_CONE_RADIUS,
			FRONT_MIDDLE_CONE_RADIUS, FRONT_MIDDLE_CUBE_ANGLE, FRONT_MIDDLE_CONE_ANGLE,
			FRONT_LOW_RADIUS,         FRONT_LOW_RADIUS,        FRONT_LOW_RADIUS
		}; //inches

		double FrontAngleValues[9] = {
			FRONT_HIGH_CONE_ANGLE,   FRONT_HIGH_CUBE_ANGLE,   FRONT_HIGH_CONE_ANGLE,
			FRONT_MIDDLE_CONE_ANGLE, FRONT_MIDDLE_CUBE_ANGLE, FRONT_MIDDLE_CONE_ANGLE,
			FRONT_LOW_ANGLE,         FRONT_LOW_ANGLE,         FRONT_LOW_ANGLE
		}; //degrees

		double BackRadiiValues[9] = {
			BACK_HIGH_CONE_RADIUS,   BACK_HIGH_CUBE_RADIUS,  BACK_HIGH_CONE_RADIUS,
			BACK_MIDDLE_CONE_RADIUS, BACK_MIDDLE_CUBE_ANGLE, BACK_MIDDLE_CONE_ANGLE,
			BACK_LOW_RADIUS,         BACK_LOW_RADIUS,        BACK_LOW_RADIUS
		}; //inches

		double BackAngleValues[9] = {
			BACK_HIGH_CONE_ANGLE,   BACK_HIGH_CUBE_ANGLE,   BACK_HIGH_CONE_ANGLE,
			BACK_MIDDLE_CONE_ANGLE, BACK_MIDDLE_CUBE_ANGLE, BACK_MIDDLE_CONE_ANGLE,
			FRONT_LOW_ANGLE,         BACK_LOW_ANGLE,         BACK_LOW_ANGLE
		}; //degrees

		int ArrayValueNeeded = (row*3) + column;

		if (isOnFrontSide) { 
			PivotToPosition(FrontAngleValues[ArrayValueNeeded]);
			Telescope(FrontRadiiValues[ArrayValueNeeded]);
		} else {
			PivotToPosition(BackAngleValues[ArrayValueNeeded]);
			Telescope(BackRadiiValues[ArrayValueNeeded]);
		}
		Squeeze(false); });
}

// Arm positions for transit
frc2::InstantCommand Arm::TransitMode()
{
	return frc2::InstantCommand([&]
								{
		PivotToPosition(TRANSIT_ANGLE);
		Telescope(TRANSIT_RADIUS); });
}

// Arm positions for picking up from the ground
frc2::InstantCommand Arm::GroundPickupMode()
{
	return frc2::InstantCommand([&]
								{
		PivotToPosition(GROUND_PICKUP_ANGLE);
		Telescope(GROUND_PICKUP_RADIUS); });
}

// Arm positions to load cone from loading station
frc2::InstantCommand Arm::LoadingMode()
{
	return frc2::InstantCommand([&]
								{
		if (isOnFrontSide) {
			PivotToPosition(FRONT_LOADING_ANGLE);
			Telescope(FRONT_LOADING_RADIUS);
		} else {
			PivotToPosition(BACK_LOADING_ANGLE);
			Telescope(BACK_LOADING_RADIUS);
		} });
}

// while override is active, gives manual joysticks control over the two arm motors
frc2::FunctionalCommand Arm::ManualControls()
{
	return frc2::FunctionalCommand([&] { // onInit
		SlipBrakes(false);
	},
	[&] { // onExecute
		m_Pivot.Set(ControlMode::PercentOutput, Robot::GetRobot()->GetJoystick().GetRawAxis(PIVOT_CONTROL) / 5);
		m_Extraction.Set(ControlMode::PercentOutput, Robot::GetRobot()->GetJoystick().GetRawAxis(EXTRACTION_CONTROL) / 5);
	},
	[&](bool e) { // onEnd
		m_Pivot.Set(ControlMode::PercentOutput, 0);
		m_Extraction.Set(ControlMode::PercentOutput, 0);
		SlipBrakes(true);
	},
	[&] { // isFinished
		return !Robot::GetRobot()->GetButtonBoard().GetRawButton(ARM_OVERRIDE);
	});
}