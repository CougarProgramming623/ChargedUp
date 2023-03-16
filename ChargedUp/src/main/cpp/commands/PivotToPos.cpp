#include "./commands/PivotToPos.h"
#include "Robot.h"

PivotToPos::PivotToPos(double angle) {
	angleToGoTo = angle;
}

void PivotToPos::Initialize() {
	Robot::GetRobot()->GetArm().GetPivotMotor().SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

	degreesToMove = angleToGoTo - Robot::GetRobot()->GetArm().GetPivotCANCoder().GetAbsolutePosition();
	ticksToMove = PivotDegToTicks(degreesToMove);

	// Robot::GetRobot()->GetArm().GetPivotMotor().Set(ControlMode::Position, ticksToMove);
}

void PivotToPos::Execute() {
	DebugOutF(std::to_string(Robot::GetRobot()->GetArm().GetPivotCANCoder().GetAbsolutePosition()));
}

void PivotToPos::End(bool interrupted) {
	Robot::GetRobot()->GetArm().GetPivotMotor().Set(ControlMode::PercentOutput, 0);
}

bool PivotToPos::IsFinished() {
	return false;//abs(PivotDegToTicks(Robot::GetRobot()->GetArm().GetPivotCANCoder().GetAbsolutePosition()) - Robot::GetRobot()->GetArm().GetPivotMotor().GetSelectedSensorPosition()) < 2;
}