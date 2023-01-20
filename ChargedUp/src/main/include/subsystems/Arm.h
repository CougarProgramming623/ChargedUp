#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>

#include <frc2/command/InstantCommand.h>



class Arm {

	public:

	Arm();
	void ArmInit();
	void ArmPower();


	private:

	ctre::phoenix::motorcontrol::can::TalonFX m_ArmPivot;
	ctre::phoenix::motorcontrol::can::TalonFX m_ClawPivot;

};