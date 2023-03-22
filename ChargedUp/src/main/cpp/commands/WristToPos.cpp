#include "./commands/WristToPos.h"
#include "Robot.h"


#define ARM Robot::GetRobot()->GetArm()
#define STRINGPOT Robot::GetRobot()->GetArm().GetPot()

WristToPos::WristToPos(double degPos) {
	targetDegree = degPos;
}

void WristToPos::Initialize() {
	ARM.GetWristMotor().SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
	targetDegree = WristTicksToDeg(targetDegree);
	ARM.GetWristMotor().Set(ControlMode::Position, targetDegree);
}

void WristToPos::Execute() {
	
}

void WristToPos::End(bool interrupted){
	ARM.GetWristMotor().Set(ControlMode::PercentOutput, 0);
}

bool WristToPos::IsFinished() {
	return abs(WristTicksToDeg(ARM.GetWristMotor().GetSelectedSensorPosition()) - targetDegree) < 2;
}