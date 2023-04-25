#include "commands/WristToPos.h"
#include "Constants.h"

WristToPos::WristToPos()
:
m_Wrist(WRIST_MOTOR)
//stuff
{}

void WristToPos::WristToPos(double angle){
    m_WristAngle = angle;
}

double WristToPos::GetWrist() {return m_WristAngle;}