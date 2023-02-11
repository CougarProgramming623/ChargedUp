#include "subsystems/Arm.h"

using ctre::phoenix::motorcontrol::ControlMode;
using ctre::phoenix::motorcontrol::can::TalonFX;

Arm::Arm() :
	m_Pivot(PIVOT_MOTOR),
	m_Extraction(EXTRACTION_MOTOR)
	// m_LeftBrake(LEFT_BRAKE),
	// m_RightBrake(RIGHT_BRAKE),
	// m_UnlockPivot([&] {return m_ButtonBoard.GetRawButton(TELE_NUKE);})
	{}

void Arm::Init() {
	// m_LeftBrake.Set(0);
	// m_RightBrake.Set(0);
	SetPID(&m_Pivot, 0, .5, .5, .5, 0); //EPIDF values are for testing
	m_Pivot.SetSelectedSensorPosition(0);
	startingTicks = m_Pivot.GetSelectedSensorPosition();

	DebugOutF("Initial encoder value");
	DebugOutF(std::to_string(startingTicks));

	SetButtons();
}

void Arm::PrintTest() {
	DebugOutF(std::to_string(m_Pivot.GetSelectedSensorPosition()));
}

void Arm::SetButtons() {
	// m_UnlockPivot.WhileHeld(frc2::InstantCommand([&] {m_Pivot.Set(ControlMode::PercentOutput, m_ButtonBoard.GetRawAxis(UP_DOWN_JOYSTICK));}));
	// m_UnlockPivot.WhenReleased(frc2::InstantCommand([&] {m_Pivot.Set(ControlMode::PercentOutput, 0);}));
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

			double currentAngle = PivotTicksToDeg(m_Pivot.GetSelectedSensorPosition() - startingTicks); //current angle of the arm
			double degToMove = angle - currentAngle; //how many degrees the arm needs to move in the correct direction
			ticksToMove = PivotDegToTicks(degToMove); //how many ticks the pivot motor needs to move in the correct direction
			setpoint = startingTicks + ticksToMove;

			DebugOutF("setpoint and current ticks:");
			DebugOutF(std::to_string(setpoint));
			DebugOutF(std::to_string(m_Pivot.GetSelectedSensorPosition()));
			
          },[&] {  // onExecute
		  	const double deadband1 = 1000; //figure out deadband
		  	const double deadband2 = 500; //figure out deadband
		  	const double deadband3 = 200; //figure out deadband

			double power;
			if(abs(m_Pivot.GetSelectedSensorPosition() - setpoint) <= deadband3) m_Pivot.Set(ControlMode::PercentOutput, 0);
			else if (abs(m_Pivot.GetSelectedSensorPosition() - setpoint) <= deadband2) power = .05;
			else if (abs(m_Pivot.GetSelectedSensorPosition() - setpoint) <= deadband1) power = .1;
			else if (abs(m_Pivot.GetSelectedSensorPosition() - setpoint) > deadband1) power = .2;

			if(setpoint > m_Pivot.GetSelectedSensorPosition()) {
				m_Pivot.Set(ControlMode::PercentOutput, power);
				DebugOutF("UNDER");
			} else if (setpoint < m_Pivot.GetSelectedSensorPosition()) {
				m_Pivot.Set(ControlMode::PercentOutput, -power);
				DebugOutF("OVER");
			}

          }, [&](bool e) {  // onEnd
			m_Pivot.Set(ControlMode::PercentOutput, 0);
			startingTicks = m_Pivot.GetSelectedSensorPosition();//increments currentTicks counter
          }, [&] {  // isFinished
			return false; //(m_Pivot.GetSelectedSensorPosition() - (ticksToMove + currentTicks) < 100); //deadband of 100 ticks 
          });
}

// Toggles the status of the brakes; currently assuming 0 is off, 1 is on
// void Arm::ToggleBrakes(bool isBraked) {
// 	if (isBraked) {
// 		m_LeftBrake.Set(1);
// 		m_RightBrake.Set(1);	
// 	} else { 
// 		m_LeftBrake.Set(0);
// 		m_RightBrake.Set(0);
// 	}
// }

//length should be a setpoint in the units the pot uses || assumes more units = longer arm || assumes forward power is increase length
void Arm::Telescope(double length) {
	// ToggleBrakes(false);
	const double telescopeDriveConstant = .5; //test to see how much power is needed
	const double deadband = -1; //figure out deadband

	if (length > m_StringPot.GetValue()) {
		while (abs(length - m_StringPot.GetValue()) > deadband) m_Extraction.Set(ControlMode::PercentOutput, telescopeDriveConstant);
		m_Extraction.Set(ControlMode::PercentOutput, 0);
	} else {
		while (abs(length - m_StringPot.GetValue()) > deadband) m_Extraction.Set(ControlMode::PercentOutput, -telescopeDriveConstant);
		m_Extraction.Set(ControlMode::PercentOutput, 0);

	}
}

//------------------------------------------------------------------------------------------------------------------- constant might be used later dont delete
// 	//angles use parallel to ground on side of arm as zero
// 	double coneAngles[] = {49.74, 44.83}; // {low target angle, high target angle}
// 	double cubeAngles[] = {30.14, 25.90}; // {low target angle, high target angle}

// 	double coneArmLengths[] = {40.62, 60.99}; //same as cone arrays (in inches)
// 	double cubeArmLengths[] = {40.83, 74.41}; // ^^^

// void Arm::LoadingReady() {
// 	Telescope(38.19);
// 	PivotToPosition(103.43);
// }