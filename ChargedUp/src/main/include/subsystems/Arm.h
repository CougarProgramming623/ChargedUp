#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
using ctre::phoenix::motorcontrol::can::TalonFX;
using ctre::phoenix::motorcontrol::can::BaseTalon;

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
    // BaseTalon m_IntakeTop;
    // BaseTalon m_IntakeBottom;
};