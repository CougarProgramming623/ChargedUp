#include "./commands/DynamicIntake.h"
#include "Robot.h"


#define ARM Robot::GetRobot()->GetArm()

DynamicIntake::DynamicIntake() {

}

void DynamicIntake::Initialize() {

}

void DynamicIntake::Execute() {
	double power = .55; //default power for cone
	if (Robot::GetRobot()->GetButtonBoard().GetRawButton(CUBE_MODE)) power *= -1; 

	if(Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_BUTTON)) ARM.GetIntakeMotor().Set(ControlMode::PercentOutput, power);
	else ARM.GetIntakeMotor().Set(ControlMode::PercentOutput, -power);
}

void DynamicIntake::End(bool interrupted){
	ARM.GetIntakeMotor().Set(ControlMode::PercentOutput, 0);
}

bool DynamicIntake::IsFinished() {
	return !Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_BUTTON) &&
		   !Robot::GetRobot()->GetButtonBoard().GetRawButton(OUTTAKE_BUTTON);
}