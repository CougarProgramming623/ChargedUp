#include "./commands/PivotToPos.h"
#include "Robot.h"


#define ARM Robot::GetRobot()->GetArm()

PivotToPos::PivotToPos(double degPos) {
	targetDegrees = degPos;
}

void PivotToPos::Initialize() {
	ARM.GetPivotMotor().SetNeutralMode(ctre::phoenix::motorcontrol::Brake);

	DebugOutF("starting at: " + std::to_string((ARM.GetPivotCANCoder().GetAbsolutePosition() - CANCODER_ZERO)) + " degrees");
	DebugOutF("Going to: " + std::to_string(ARM.PivotTicksToDegrees(-ARM.PivotDegreesToTicks(targetDegrees))) + " degrees");

}

void PivotToPos::Execute() {
	ARM.GetPivotMotor().Set(ControlMode::Position, -ARM.PivotDegreesToTicks(targetDegrees));
	// DebugOutF("NOT YET FINISHED.");
}

void PivotToPos::End(bool interrupted){
	ARM.GetPivotMotor().Set(ControlMode::PercentOutput, 0);
}

bool PivotToPos::IsFinished() {
	return abs(ARM.PivotDegreesToTicks(targetDegrees) - ARM.GetPivotMotor().GetSelectedSensorPosition()) < 4000;
}