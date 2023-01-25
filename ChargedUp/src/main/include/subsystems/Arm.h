#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/Joystick.h>
#include <frc/Servo.h>
#include <ctre/phoenix/motorcontrol/can/BaseMotorController.h>


#include <frc2/command/InstantCommand.h>


using ctre::phoenix::motorcontrol::can::TalonFX;

class Arm {

	public:

	Arm();
	void ArmInit();
	inline double DegToTicks(double degree) {return degree * PIVOT_TICKS_PER_ARM_DEGREE;} //converts degrees to ticks of Pivot motor
	inline double TicksToDeg(double ticks) {return ticks / PIVOT_TICKS_PER_ARM_DEGREE;} //converts ticks to degrees of arm rotation
	void PivotToPosition(int angle);
	void EnableBrakes();
	void DisableBrakes();

	private:

	const int PIVOT_GEAR_RATIO = 320 / 1;
	const double PIVOT_TICKS_PER_ARM_DEGREE = 1820.444;



	TalonFX m_Pivot;
	TalonFX m_Extraction;
	frc::Servo m_LeftBrake;
	frc::Servo m_RightBrake;
	

};