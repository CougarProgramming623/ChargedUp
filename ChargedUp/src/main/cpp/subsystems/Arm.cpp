#include "subsystems/Arm.h"

using ctre::phoenix::motorcontrol::ControlMode;


Arm::Arm() :
	m_Pivot(0),
	m_Extraction(0),
	m_LeftBrake(0),
	m_RightBrake(0)
	{}

void Arm::ArmInit() {
	m_Pivot.SetSelectedSensorPosition(0); //sets the initial position to 0 ticks- arm starting position will be where all angles are relative to
	m_LeftBrake.Set(1);
	m_RightBrake.Set(1);
}


//Moves the arm to a set position
void Arm::PivotToPosition(int angle) {
	double currentAngle = TicksToDeg(m_Pivot.GetSelectedSensorPosition()); //current angle of the arm
	double degToMove = angle - currentAngle; //how many degrees the arm needs to move in the correct direction
	double ticksToMove = DegToTicks(degToMove); //how many ticks the pivot motor needs to move in the correct direction

	m_Pivot.Set(ControlMode::Position, ticksToMove);
} 

//Brakes the arms
void Arm::EnableBrakes() {
	m_LeftBrake.Set(0);
	m_RightBrake.Set(0);	

}

//Unbrakes the arms
void Arm::DisableBrakes() {
	m_LeftBrake.Set(1);
	m_RightBrake.Set(1);
}