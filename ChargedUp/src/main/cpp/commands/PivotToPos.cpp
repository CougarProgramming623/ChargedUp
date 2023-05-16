#include "commands/PivotToPos.h"
#include "Constants.h"

PivotToPos::PivotToPos()
m_Pivot(PIVOT_MOTOR)
//stuff
{
    //requirements?
}

void PivotToPos::PivotToPos(int angle){
    m_PivotAngle = angle;
}
void PivotToPos::Initialize() {
    DebugOutF("Initialized");
}

void PivotToPos::Execute() {
    //m_Pivot.Set(ControlMode::MotionMagic, Robot::GetRobot().GetArm().m_OffsetTicks)
    // motion magic - going from m_CurrentDeg to m_PivotAngle
}


double PivotToPos::GetPivot() {return m_PivotAngle;}