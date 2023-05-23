#include "commands/WristToPos.h"
#include "Constants.h"

WristToPos::WristToPos(double angle){
    m_WristAngle = angle;
}

double WristToPos::GetWrist() {return m_WristAngle;}