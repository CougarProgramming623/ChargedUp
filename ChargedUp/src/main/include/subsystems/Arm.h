#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/Joystick.h>
#include <frc/Servo.h>
#include <ctre/phoenix/motorcontrol/can/BaseMotorController.h>
#include <frc/Joystick.h>
#include <frc2/command/button/Button.h>
#include <frc/AnalogInput.h>


#include <math.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "Constants.h"
#include "Util.h"
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/ParallelCommandGroup.h>


using ctre::phoenix::motorcontrol::can::TalonFX;

class Arm : public frc2::SubsystemBase {

	public:

	Arm();
	void Init();
	void SetButtons();

	//conversions
	inline double PivotDegToTicks(double degree) {return degree * PIVOT_TICKS_PER_ARM_DEGREE;} //converts degrees to ticks of Pivot motor
	inline double PivotTicksToDeg(double ticks) {return ticks / PIVOT_TICKS_PER_ARM_DEGREE;} //converts ticks to degrees of arm rotation
	inline double StringPotUnitsToInches(double units) {return (units - STRING_POT_MINIMUM) * STRING_POT_INCHES_PER_TICK;} //166 = length of slider
	inline double InchesToStringPotUnits(double inches) {return inches / STRING_POT_INCHES_PER_TICK;}
	//basic commands
	frc2::InstantCommand* PivotToPosition(double angle); 
	void ArmBrakes(bool shouldBreak);
	void SlipBrakes(bool shouldBreak);
	frc2::SequentialCommandGroup* WaitBrakeTelescope(double setpoint);
	frc2::FunctionalCommand* Telescope(double setpoint); 
	frc2::SequentialCommandGroup* Squeeze();
	//Automation
	frc2::SequentialCommandGroup* PlaceElement(int row, int column);
	frc2::SequentialCommandGroup* TransitMode();
	frc2::SequentialCommandGroup* GroundPickupMode();
	frc2::SequentialCommandGroup* LoadingMode();
	//misc
	frc2::FunctionalCommand ManualControls();

	inline frc::AnalogInput& GetPot() { return m_StringPot; }
	inline void PrintPot() {DebugOutF(std::to_string(m_StringPot.GetValue()));}
	inline TalonFX& GetPivot() {return m_Pivot; }

	int SelectedRow;
	int SelectedColumn;

	bool shouldSqueeze;

	private:
	
	//class constants
	bool isOnFrontSide = false; //switch will flip this boolean to change method behaviour
	
	
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
	TalonFX m_Pivot; //Positive drives towards back; negative drives towards front || Start at 0.1-0.2 power and scale from there while testing
	TalonFX m_Extraction; //Positive drives arm together and in; negative drives arm apart and out || start at 0.1 power and scale from there while testing

	//Servos
	frc::Servo m_LeftBrake;
	frc::Servo m_RightBrake;
	frc::Servo m_SlipBrake;

	//Pot
	frc::AnalogInput m_StringPot{STRINGPOT_ANALOG_INPUT_ID};

	//buttons
	frc2::Button m_Squeeze;
	
	frc2::Button m_TL;
	frc2::Button m_TC;
	frc2::Button m_TR;
	frc2::Button m_ML;
	frc2::Button m_MC;
	frc2::Button m_MR;
	frc2::Button m_BL;
	frc2::Button m_BC;
	frc2::Button m_BR;

	frc2::Button m_LeftGrid;
	frc2::Button m_CenterGrid;
	frc2::Button m_RightGrid;

	frc2::Button m_TransitMode;
	frc2::Button m_GroundPickupMode;
	frc2::Button m_LoadingMode;

	frc2::Button m_Override;

	frc2::Button m_ConeMode;
	frc2::Button m_CubeMode;

	frc2::Button m_FrontMode;
	frc2::Button m_BackMode;

	frc2::Button m_ManualArmBrake;
	frc2::Button m_ManualSlipBrake;

	frc::Timer m_Timer;

	frc2::SequentialCommandGroup* m_Top;
	frc2::SequentialCommandGroup* m_Mid;
	frc2::SequentialCommandGroup* m_Bot;
};