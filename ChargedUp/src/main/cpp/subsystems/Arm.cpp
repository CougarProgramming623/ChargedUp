#include "subsystems/Arm.h"

using ctre::phoenix::motorcontrol::ControlMode;

Arm::Arm() :
	m_Pivot(PIVOT_MOTOR),
	m_Extraction(EXTRACTION_MOTOR),
	m_LeftBrake(LEFT_BRAKE),
	m_RightBrake(RIGHT_BRAKE)
	{}

void Arm::ArmInit() {
	m_Pivot.SetSelectedSensorPosition(0); //sets the initial position to 0 ticks- arm starting position will be where all angles are relative to
	m_LeftBrake.Set(0);
	m_RightBrake.Set(0);
}


//Moves the arm to a set position
void Arm::PivotToPosition(double angle) {
	double currentAngle = PivotTicksToDeg(m_Pivot.GetSelectedSensorPosition()); //current angle of the arm
	double degToMove = angle - currentAngle; //how many degrees the arm needs to move in the correct direction
	double ticksToMove = PivotDegToTicks(degToMove); //how many ticks the pivot motor needs to move in the correct direction

	m_Pivot.Set(ControlMode::Position, ticksToMove);
} 

//Toggles the status of the brakes; currently assuming 0 is off, 1 is on
void Arm::ToggleBrakes() {
	if (m_brakesActive) {
		m_LeftBrake.Set(0);
		m_RightBrake.Set(0);	
	} else { 
		m_LeftBrake.Set(1);
		m_RightBrake.Set(1);
	}
}

//Changes the length of the arm; positive parameter is longer, negative parameter is shorter || currently assuming forward (positive) direction is extension
void Arm::Telescope(double length) {

	double ticksPerBarRotation = EXTRACTION_GEAR_RATIO * 2048;
	double tickPerInch = ticksPerBarRotation / EXTRACTION_BAR_CIRCUMFERENCE;
	double ticksToMove = tickPerInch * (length - currentLength);
	double setpoint = m_Extraction.GetSelectedSensorPosition() + ticksToMove;

	if (m_brakesActive) ToggleBrakes();

	m_Pivot.Set(ControlMode::Position, setpoint);
}

//Toggles if the arm is squeezing shut or not to hold game pieces
void Arm::Squeeze(bool shouldSqueeze) {
	double squeezeConstant = 1; //scalar for amount of power needed to squeeze || should be 0-1
	if (!shouldSqueeze) squeezeConstant * -1; //inverts power to release

	if (!m_brakesActive) ToggleBrakes();

	m_Extraction.Set(ControlMode::PercentOutput, squeezeConstant);
}

//Should be called when arm is holding a game piece and robot is ready to drop it. Level is 0 as the lowest and 1 as the highest
void Arm::AutoDrop(bool isCone, int level) {

	//angles use parallel to ground on side of arm as zero
	double coneAngles[] = {49.74, 44.83}; // {low target angle, high target angle}
	double cubeAngles[] = {30.14, 25.90}; // {low target angle, high target angle}

	double coneArmLengths[] = {40.62, 60.99}; //same as cone arrays (in inches)
	double cubeArmLengths[] = {40.83, 74.41}; // ^^^

	if(isCone) {
		Telescope(coneArmLengths[level]);
		PivotToPosition(coneAngles[level]);
		Squeeze(false);
	} else {
		Telescope(cubeArmLengths[level]);
		PivotToPosition(cubeAngles[level]);
		Squeeze(false);
	}
}

void Arm::LoadingReady() {
	Telescope(38.19);
	PivotToPosition(103.43);
}