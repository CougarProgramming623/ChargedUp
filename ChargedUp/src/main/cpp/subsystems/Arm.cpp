#include "subsystems/Arm.h"

using ctre::phoenix::motorcontrol::ControlMode;
using ctre::phoenix::motorcontrol::can::TalonFX;

Arm::Arm() :
	m_Pivot(PIVOT_MOTOR),
	m_Extraction(EXTRACTION_MOTOR),
	m_LeftBrake(LEFT_BRAKE),
	m_RightBrake(RIGHT_BRAKE),

	m_TL(BUTTON_L(GRID_TL)),
	m_TC(BUTTON_L(GRID_TC)),
	m_TR(BUTTON_L(GRID_TR)),
	m_ML(BUTTON_L(GRID_ML)),
	m_MC(BUTTON_L(GRID_MC)),
	m_MR(BUTTON_L(GRID_MR)),
	m_BL(BUTTON_L(GRID_BL)),
	m_BC(BUTTON_L(GRID_BC)),
	m_BR(BUTTON_L(GRID_BR)),

	m_LeftGrid(BUTTON_L(LEFT_GRID)),
	m_CenterGrid(BUTTON_L(CENTER_GRID)),
	m_RightGrid(BUTTON_L(RIGHT_GRID)),

	m_ConeMode(BUTTON_L(CONE_MODE)),
	m_CubeMode(BUTTON_L(CUBE_MODE)),

	m_FrontMode(BUTTON_L(FRONT_MODE)),
	m_BackMode(BUTTON_L(BACK_MODE)),

	m_TransitMode(BUTTON_L(TRANSIT_MODE)),
	m_GroundPickupMode(BUTTON_L(GROUND_PICKUP_MODE)),
	m_LoadingMode(BUTTON_L(LOADING_MODE))
	{}

void Arm::Init() {
	m_Pivot.SetSelectedSensorPosition(0);
	SetButtons();
	ToggleBrakes(true);
	m_Pivot.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	m_Extraction.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

}



void Arm::SetButtons() {
//empty
}


frc2::FunctionalCommand Arm::PivotToPosition(double angleSetpoint) {
	Angle = angleSetpoint;
	return frc2::FunctionalCommand(
          [&] {  // onInit
			StartingTicks = m_Pivot.GetSelectedSensorPosition();
			double currentAngle = PivotTicksToDeg(m_Pivot.GetSelectedSensorPosition()); //current angle of the arm
			double degToMove = Angle - currentAngle; //how many degrees the arm needs to move in the correct direction
			TicksToMove = PivotDegToTicks(degToMove); //how many ticks the pivot motor needs to move in the correct direction
			Setpoint = StartingTicks + TicksToMove;
          },[&] {  // onExecute		  	
			m_Pivot.Set(ControlMode::Position, Setpoint);
          }, [&](bool e) {  // onEnd
			m_Pivot.Set(ControlMode::PercentOutput, 0);
		  }, [&] {  // isFinished
			return (abs(m_Pivot.GetSelectedSensorPosition() - Setpoint) < 200); //deadband of 100 ticks 
		  });
}

// Toggles the status of the brakes; currently assuming 0 is off, 1 is on
void Arm::ToggleBrakes(bool isBraked) {
	if (isBraked) {
		m_LeftBrake.Set(1);
		m_RightBrake.Set(1);	
	} else { 
		m_LeftBrake.Set(0);
		m_RightBrake.Set(0);
	}
	m_brakesActive = isBraked;
}

//length should be a setpoint in inches || assumes forward power is increase length
frc2::FunctionalCommand Arm::Telescope(double Setpoint) {
	SetpointLength = Setpoint;
	return frc2::FunctionalCommand(
		[&] {  // onInit
		ToggleBrakes(false);
		DebugOutF(std::to_string(SetpointLength));
		},[&] {  // onExecute		  	
		ArmLength = StringPotUnitsToInches(m_StringPot.GetValue());

		if(ArmLength < SetpointLength) m_Extraction.Set(ControlMode::PercentOutput, .5);
		else if(ArmLength > SetpointLength) m_Extraction.Set(ControlMode::PercentOutput, -.5);

		}, [&](bool e) {  // onEnd
			ToggleBrakes(true);
			m_Extraction.Set(ControlMode::PercentOutput, 0);
		}, [&] {  // isFinished
			return (abs(ArmLength - SetpointLength) < .01);
		});
}

frc2::FunctionalCommand Arm::Squeeze(bool shouldSqueeze) {	
	if(shouldSqueeze) {
		return frc2::FunctionalCommand( [&] { //onInit
			TicksToUndoSqueeze = m_Extraction.GetSelectedSensorPosition();
			m_Extraction.Set(ControlMode::PercentOutput, 1);
		}, [&] {//onExecute
			//empty
		}, [&](bool e) {//onEnd
			m_Extraction.Set(ControlMode::PercentOutput, 0);
			TicksToUndoSqueeze = TicksToUndoSqueeze - m_Extraction.GetSelectedSensorPosition();
		}, [&] {//isFinished
			return m_Extraction.GetSupplyCurrent() < SQUEEZE_AMP_THRESHOLD;
		});}
	else {
		return frc2::FunctionalCommand( [&] { //onInit
			TicksToUndoSqueeze = m_Extraction.GetSelectedSensorPosition() - TicksToUndoSqueeze;
			m_Extraction.Set(ControlMode::Position, TicksToUndoSqueeze);
		}, [&] {//onExecute
			//empty
		}, [&](bool e) {//onEnd
			m_Extraction.Set(ControlMode::PercentOutput, 0);
		}, [&] {//isFinished
			return m_Extraction.GetSelectedSensorPosition() - TicksToUndoSqueeze  < 100; 
		});
		}
	}

//sets arm to a set angle and radius based on element being places; rows and column read top to bottom and left to right respectively
//type can be either CONE or CUBE
void Arm::PlaceElement(int type, int row, int column) {
	//index 0 is TL; index 8 is BR (read like a book)
	double RadiiValues[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; //inches
	double AngleValues[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; //degrees
	int ArrayValueNeeded = (row*3) + column;

	PivotToPosition(AngleValues[ArrayValueNeeded]);
	Telescope(RadiiValues[ArrayValueNeeded]);
	if(type = CONE) PivotToPosition(AngleValues[ArrayValueNeeded]-2); //get cone closer to pole
	Squeeze(false);
	if(type = CONE) PivotToPosition(AngleValues[ArrayValueNeeded]+2);
}

//Gets arm as short as possible and in the bot
void Arm::TransitMode() {
	PivotToPosition(145);
	Telescope(0);
}

//Gets arm ready to load from shelf || boolean represents if loading should be from the same side as the arm is mounted 
void Arm::LoadReady(bool isOnSameSide) {
	if (isOnSameSide) {
		PivotToPosition(29.8728516038);
		Telescope(65);
	} else {
		PivotToPosition(150.127148396);
		Telescope(65);
	}
}