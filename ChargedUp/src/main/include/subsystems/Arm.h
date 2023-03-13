#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/Joystick.h>
#include <frc/Servo.h>
#include <ctre/phoenix/motorcontrol/can/BaseMotorController.h>
#include <frc/Joystick.h>
#include <frc2/command/button/Button.h>
#include <frc/AnalogInput.h>
#include <math.h>
#include <ctre/phoenix/sensors/CANCoder.h>

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
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "./commands/PivotToPos.h"



using ctre::phoenix::motorcontrol::can::TalonFX;
using ctre::phoenix::motorcontrol::can::TalonSRX;


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

	frc2::FunctionalCommand* ManualControls();

	inline frc::AnalogInput& GetPot() { return m_StringPot; }
	inline void PrintPot() {DebugOutF(std::to_string(m_StringPot.GetValue()));}
	
	inline TalonSRX& GetPivotMotor() {return m_Pivot;}
	inline TalonSRX& GetWristMotor() {return m_Wrist;} 
	inline TalonSRX& GetIntakeMotor() {return m_Intake;}
	inline ctre::phoenix::sensors::CANCoder& GetPivotCANCoder() {return m_PivotCANCoder;}

	private:

	//motors
	TalonSRX m_Pivot; 
	ctre::phoenix::sensors::CANCoder m_PivotCANCoder{PIVOT_CAN_ID};

	TalonSRX m_Wrist; 

	TalonSRX m_Intake;

	//Servos

	//Pot
	frc::AnalogInput m_StringPot{STRINGPOT_ANALOG_INPUT_ID};

	//buttons
	frc2::Button m_TransitMode;
	frc2::Button m_GroundPickupMode;
	frc2::Button m_LoadingMode;

	frc2::Button m_Override;
	frc2::Button m_Override2;

	frc2::Button m_ConeMode;
	frc2::Button m_CubeMode;

	frc::Timer m_Timer;

	frc2::SequentialCommandGroup* m_Top;
	frc2::SequentialCommandGroup* m_Mid;
	frc2::SequentialCommandGroup* m_Bot;
};