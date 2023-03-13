#include "./commands/PivotToPos.h"
#include "Robot.h"

PivotToPos::PivotToPos(double angle) {
	angleToGoTo = angle;
}

void PivotToPos::Initialize() {
	Robot::GetRobot()->GetArm().GetPivotMotor().SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

	degreesToMove = angleToGoTo - Robot::GetRobot()->GetArm().GetPivotCANCoder().GetAbsolutePosition() + PIVOT_CAN_OFFSET;
	ticksToMove = Robot::GetRobot()->GetArm().PivotDegToTicks(degreesToMove);

	Robot::GetRobot()->GetArm().GetPivotMotor().Set(ControlMode::Position, ticksToMove);
}

void PivotToPos::End(bool interrupted) {
	Robot::GetRobot()->GetArm().GetPivotMotor().Set(ControlMode::PercentOutput, 0);
}

bool PivotToPos::IsFinished() {
	return abs(Robot::GetRobot()->GetArm().PivotDegToTicks(Robot::GetRobot()->GetArm().GetPivotCANCoder().GetAbsolutePosition()) - Robot::GetRobot()->GetArm().GetPivotMotor().GetSelectedSensorPosition()) < 2;
}