#include "commands/PivotToPos.h"
#include "Constants.h"

PivotToPos::PivotToPos()
:
m_Pivot(PIVOT_MOTOR)
//stuff
{}

void PivotToPos::PivotToPos(double angle){
    m_PivotAngle = angle;
}

double PivotToPos::GetPivot() {return m_PivotAngle;}