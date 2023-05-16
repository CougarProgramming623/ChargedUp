#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
//include stuff

class Arm : public frc2::SubsystemBase {
    public:

    Arm();
    void ArmInit();

    int m_TicksToDeg(int ticks);
    int m_DegToTicks(int deg);

    int m_OffsetTicks = -1897;

    TalonFX m_Wrist;
    TalonFX m_Pivot;
};