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
	void SetButtons();

	inline double PivotDegToTicks(double degree) {return degree * PIVOT_TICKS_PER_ARM_DEGREE;} //converts degrees to ticks of Pivot motor
	inline double PivotTicksToDeg(double ticks) {return ticks / PIVOT_TICKS_PER_ARM_DEGREE;} //converts ticks to degrees of arm rotation
	inline double StringPotUnitsToInches(double units) {return (units - 166) * STRING_POT_INCHES_PER_TICK;} //166 = length of slider
	inline double InchesToStringPotUnits(double inches) {return inches / STRING_POT_INCHES_PER_TICK;}

	frc2::FunctionalCommand PivotToPosition(double angle); 
	void ToggleBrakes(bool isBraked); 

	frc2::FunctionalCommand Telescope(double setpoint); //3988 - 4058 +-2 on both bounds?

	frc2::FunctionalCommand Squeeze(bool shouldSqueeze);

	private:
	
	//class constants
	bool m_brakesActive = false;
	double TicksToUndoSqueeze = 0;
	
	double StartingTicks; //current ticks of encoder after movement
	double Setpoint;
	double TicksToMove;
	double Angle;

	double SetpointLength;
	double ArmLength;

	TalonFX m_Pivot;
	TalonFX m_Extraction;

	frc::AnalogInput m_StringPot{STRINGPOT_ANALOG_INPUT_ID};

	frc::Servo m_LeftBrake;
	// frc::Servo m_RightBrake;
	
	// frc::Joystick m_ButtonBoard = frc::Joystick(0);
	frc::Joystick m_Joystick = frc::Joystick(1);
	frc2::Button m_TestJoystickButton;

};