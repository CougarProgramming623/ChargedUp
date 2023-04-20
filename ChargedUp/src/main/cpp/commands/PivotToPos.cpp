#include "commands/PivotToPos.h"
#include "Constants.h"

PivotToPos::PivotToPos()
:
m_Pivot(PIVOT_MOTOR)

//stuff
{

}


double PivotToPos::GetPivot() {return m_PivotAngle;}