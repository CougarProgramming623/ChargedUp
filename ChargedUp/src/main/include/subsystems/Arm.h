#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/Joystick.h>
#include <frc/Servo.h>
#include <ctre/phoenix/motorcontrol/can/BaseMotorController.h>
#include <frc/Joystick.h>
#include <frc2/command/button/Button.h>
#include <frc/AnalogInput.h>

#include <math.h>


#include <frc2/command/PrintCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "Constants.h"
#include "Util.h"




using ctre::phoenix::motorcontrol::can::TalonFX;

class Arm {

	public:

	Arm();
	void Init();
	void PrintTest();

	void SetButtons();

	inline double PivotDegToTicks(double degree) {return degree * PIVOT_TICKS_PER_ARM_DEGREE;} //converts degrees to ticks of Pivot motor
	inline double PivotTicksToDeg(double ticks) {return ticks / PIVOT_TICKS_PER_ARM_DEGREE;} //converts ticks to degrees of arm rotation
	void SetPID(TalonFX* motor, double E, double P, double I, double D, double F);
	
	frc2::FunctionalCommand* PivotToPosition(double angle); 
	void ToggleBrakes(bool isBraked); 

	void Telescope(double length); 

	private:
	
	bool m_brakesActive = false;
	double startingTicks; //current ticks of encoder after movement
	double setpoint;
	double ticksToMove;

	TalonFX m_Pivot;
	TalonFX m_Extraction;

	frc::AnalogInput m_StringPot{STRINGPOT_ANALOG_INPUT_ID};

	// frc::Servo m_LeftBrake;
	// frc::Servo m_RightBrake;
	
	// frc::Joystick m_ButtonBoard = frc::Joystick(0);
	// frc2::Button m_UnlockPivot;

};