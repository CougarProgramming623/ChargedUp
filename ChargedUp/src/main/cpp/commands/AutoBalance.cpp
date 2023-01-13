#include "commands/AutoBalance.h"
#include "Robot.h"

AutoBalance:: AutoBalance() {
    //AddRequirements(&Robot::GetRobot()->GetDriveTrain());
}

void AutoBalance::Initialize() {
  
}

void AutoBalance::Execute() {

    double m_angle = Robot::GetRobot()->GetNavX().GetPitch();

    double m_error = 0 - m_angle;


  
}