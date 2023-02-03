#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/Joystick.h>
#include <frc/Servo.h>
#include <ctre/phoenix/motorcontrol/can/BaseMotorController.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>


#include "Constants.h"
#include "Util.h"

#include <frc/Joystick.h>
#include <frc2/command/button/Button.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/FunctionalCommand.h>


using ctre::phoenix::motorcontrol::can::TalonFX;

class Arm {

	public:

	Arm();
	void Init();

	inline double PivotDegToTicks(double degree) {return degree * PIVOT_TICKS_PER_ARM_DEGREE;} //converts degrees to ticks of Pivot motor
	inline double PivotTicksToDeg(double ticks) {return ticks / PIVOT_TICKS_PER_ARM_DEGREE;} //converts ticks to degrees of arm rotation
	
	void PivotToPosition(double angle); 
	frc2::FunctionalCommand* PivotToPositionNew(double angle); 
	// void ToggleBrakes(); 

	void Telescope(double length); 
	void Squeeze (bool shouldSqueeze);

	//automation methods below
	void AutoDrop(bool isCone, int level);
	void LoadingReady();

	void TurnFifteen();

	private:

	bool m_brakesActive = false;
	double currentLength = -1; //NOT CORRECT- NEEDS TO BE UPDATED ||should initialize as the minimum length of the arm
	

	TalonFX m_Pivot;
	TalonFX m_Extraction;
	//frc::Servo m_LeftBrake;
	//frc::Servo m_RightBrake;
	
	frc::Joystick m_ButtonBoard = frc::Joystick(0);
	frc2::Button m_Button;

};