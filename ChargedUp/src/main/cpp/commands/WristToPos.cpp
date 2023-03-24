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
	ARM.GetWristMotor().SetSelectedSensorPosition((ARM.WristStringPotUnitsToTicks(ARM.GetStringPot().GetValue()))-29000.0- ARM.WristDegreesToTicks(45));
	ARM.GetWristMotor().Set(ControlMode::Position, ARM.WristDegreesToTicks(targetDegrees));
	// DebugOutF("TargetDeg: " + std::to_string(targetDegrees));
	// DebugOutF("TargetTicks: " + std::to_string(ARM.WristDegreesToTicks(targetDegrees)));
	


}

void WristToPos::End(bool interrupted){
	ARM.GetWristMotor().Set(ControlMode::PercentOutput, 0);
}

bool WristToPos::IsFinished() {
	//return abs(ARM.WristDegreesToTicks(targetDegrees) - ARM.GetWristMotor().GetSelectedSensorPosition()) < 20000;
	return Robot::GetRobot()->GetButtonBoard().GetRawButton(ARM_OVERRIDE);
;
}