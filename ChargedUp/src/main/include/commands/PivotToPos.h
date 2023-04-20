#pragma once

#include "Util.h"

#include <ctre/phoenix/motorcontrol/NeutralMode.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/AnalogInput.h>


using ctre::phoenix::motorcontrol::can::TalonFX;
using ctre::phoenix::motorcontrol::ControlMode;

class PivotToPos {
    public:
        PivotToPos();

        double GetPivot();

        double m_PivotAngle;
        TalonFX m_Pivot;

};