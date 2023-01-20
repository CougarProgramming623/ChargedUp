#include "subsystems/Arm.h"

using ctre::phoenix::motorcontrol::ControlMode;

Arm::Arm() :
	m_ArmPivot(0),
	m_ClawPivot(0)
 {}

void Arm::ArmInit() {
}

void Arm::ArmPower() {
	m_ArmPivot.Set(ControlMode::PercentOutput, 1);
}