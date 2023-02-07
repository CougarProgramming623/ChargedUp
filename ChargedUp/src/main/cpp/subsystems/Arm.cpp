#include "subsystems/Arm.h"

using ctre::phoenix::motorcontrol::ControlMode;
using ctre::phoenix::motorcontrol::can::TalonFX;

Arm::Arm() :
	m_Pivot(PIVOT_MOTOR),
	m_Extraction(EXTRACTION_MOTOR),	
	//m_LeftBrake(LEFT_BRAKE),
	//m_RightBrake(RIGHT_BRAKE),

	m_Button([&] {return m_ButtonBoard.GetRawButton(13);})
	{}

void Arm::Init() {
	// m_LeftBrake.Set(0);
	// m_RightBrake.Set(0);
	// SetPID(&m_Pivot, 0, .1, .1, .1, 0); //EPIDF values are for testing
	startingTicks = m_Pivot.GetSelectedSensorPosition();
	currentTicks = m_Pivot.GetSelectedSensorPosition();
}

//sets the EPIDF values for motor provided
void Arm::SetPID(TalonFX* motor, double E, double P, double I, double D, double F) {
	motor->ConfigAllowableClosedloopError(0.0, E, 0.0);
	motor->Config_kP(0.0, P, 0.0);
	motor->Config_kI(0.0, I, 0.0);
	motor->Config_kD(0.0, D, 0.0);
	motor->Config_kF(0.0, F, 0.0);
}

frc2::FunctionalCommand* Arm::PivotToPosition(double angle) {
	return new frc2::FunctionalCommand(
          [&] {  // onInit
			DebugOutF("Running Functional Command");
          },[&] {  // onExecute
		  	double currentAngle = PivotTicksToDeg(m_Pivot.GetSelectedSensorPosition() - startingTicks); //current angle of the arm
			double degToMove = angle - currentAngle; //how many degrees the arm needs to move in the correct direction
			ticksToMove = PivotDegToTicks(degToMove); //how many ticks the pivot motor needs to move in the correct direction

			m_Pivot.Set(ControlMode::Position, ticksToMove + startingTicks); 
          }, [&](bool e) {  // onEnd
			m_Pivot.Set(ControlMode::PercentOutput, 0);
			startingTicks = m_Pivot.GetSelectedSensorPosition();//increments currentTicks counter
          }, [&] {  // isFinished
			return false; //(m_Pivot.GetSelectedSensorPosition() - (ticksToMove + currentTicks) < 100); //deadband of 100 ticks 
          });
}

//Toggles the status of the brakes; currently assuming 0 is off, 1 is on
// void Arm::ToggleBrakes() {
// 	if (m_brakesActive) {
// 		m_LeftBrake.Set(0);
// 		m_RightBrake.Set(0);	
// 	} else { 
// 		m_LeftBrake.Set(1);
// 		m_RightBrake.Set(1);
// 	}
// }

//Changes the length of the arm; positive parameter is longer, negative parameter is shorter || currently assuming forward (positive) direction is extension
void Arm::Telescope(double length) {

	double ticksPerBarRotation = EXTRACTION_GEAR_RATIO * 2048;
	double tickPerInch = ticksPerBarRotation / EXTRACTION_BAR_CIRCUMFERENCE;
	double ticksToMove = tickPerInch * (length - currentLength);
	double setpoint = m_Extraction.GetSelectedSensorPosition() + ticksToMove;

	// if (m_brakesActive) ToggleBrakes();

	m_Pivot.Set(ControlMode::Position, setpoint);
}

//Toggles if the arm is squeezing shut or not to hold game pieces
void Arm::Squeeze(bool shouldSqueeze) {
	double squeezeConstant = 1; //scalar for amount of power needed to squeeze || should be 0-1
	if (!shouldSqueeze) squeezeConstant * -1; //inverts power to release

	// if (!m_brakesActive) ToggleBrakes();

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

//rotates the wheel 15 degrees when button is pressed || TESTING PURPOSES ONLY
void Arm::TurnFifteen() {
	
	m_Button.WhenPressed(frc2::InstantCommand([&]{("Yo this shit is working so slay amazing");}));
		//frc2::InstantCommand([&] {PivotToPosition(15);})),
	
}