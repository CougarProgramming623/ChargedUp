#include "./commands/WristToPos.h"
#include "Robot.h"


#define ARM Robot::GetRobot()->GetArm()
#define CURRENT_DEGREE ARM.StringPotUnitsToDeg(ARM.GetPot().GetValue())

WristToPos::WristToPos(double degPos) {
	targetDegree = degPos;
}

void WristToPos::Initialize() {
	ARM.GetWristMotor().SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
}

void WristToPos::Execute() {
	double power = .2;
	if(targetDegree < CURRENT_DEGREE) power *= -1;

	ARM.GetWristMotor().Set(ControlMode::PercentOutput, power);
}

void WristToPos::End(bool interrupted){
	ARM.GetWristMotor().Set(ControlMode::PercentOutput, 0);
}

bool WristToPos::IsFinished() {
	return true; //abs(CURRENT_DEGREE - targetDegree) < 5; //ADD THE STRING POT UNITS TO DEGREES METHODS!!! THIS WILL BREAK SHITS
}