#include "./commands/WristToPos.h"
#include "Robot.h"


#define ARM Robot::GetRobot()->GetArm()

WristToPos::WristToPos(double degPos) {
	targetDegrees = degPos;
	AddRequirements(&Robot::GetRobot()->GetArm());
}

void WristToPos::Initialize() {
	ARM.GetWristMotor().SetNeutralMode(ctre::phoenix::motorcontrol::Brake);

}

void WristToPos::Execute() {
	ARM.GetWristMotor().Set(ControlMode::Position, ARM.WristDegreesToTicks(targetDegrees));
	DebugOutF("NOT YET FINISHED.");

}

void WristToPos::End(bool interrupted){
	ARM.GetWristMotor().Set(ControlMode::PercentOutput, 0);
}

bool WristToPos::IsFinished() {
	//return abs(ARM.WristDegreesToTicks(targetDegrees) - ARM.GetWristMotor().GetSelectedSensorPosition()) < 20000;
	return false;
}