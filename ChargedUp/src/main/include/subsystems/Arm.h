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
	inline double StringPotUnitsToInches(double units) {return (units - 166) * SLIDER_INCHES_PER_TICK;} //166 = length of slider
	inline double InchesToStringPotUnits(double inches) {return inches / SLIDER_INCHES_PER_TICK;}

	frc2::FunctionalCommand PivotToPosition(double angle); 
	void ToggleBrakes(bool isBraked); 

	frc2::FunctionalCommand Telescope(double setpoint); //3988 - 4058 +-2 on both bounds?

	frc2::FunctionalCommand Squeeze(bool shouldSqueeze);
	void PlaceElement(int type, int row, int column = 1);
	void TransitMode();
	void LoadReady(bool isOnSameSide);

	private:
	
	//class constants
	bool m_brakesActive = false;
	
	
	//PivotToPosition()
	double StartingTicks; //current ticks of encoder after movement
	double Setpoint;
	double TicksToMove;
	double Angle;

	//Telescope()
	double SetpointLength;
	double ArmLength;

	//Squeeze()
	double TicksToUndoSqueeze = 0;

	//motors
	TalonFX m_Pivot;
	TalonFX m_Extraction;

	//Servos
	frc::Servo m_LeftBrake;
	frc::Servo m_RightBrake;

	//Pot
	frc::AnalogInput m_StringPot{STRINGPOT_ANALOG_INPUT_ID};

	
	// frc::Joystick m_ButtonBoard = frc::Joystick(0);
	frc::Joystick m_Joystick = frc::Joystick(1);
	frc2::Button m_TestJoystickButton;

};