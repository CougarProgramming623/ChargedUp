#pragma once

#include "Util.h"

#include <ctre/phoenix/motorcontrol/NeutralMode.h>
#include "frc/motorcontrol/PWMMotorController.h
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/AnalogInput.h>

using ctre::phoenix::motorcontrol::can::BaseTalon;
using ctre::phoenix::motorcontrol::can::TalonFX;
using ctre::phoenix::motorcontrol::ControlMode;

class DynamicIntake {
    public:
        DynamicIntake();

        BaseTalon m_IntakeTop;
        BaseTalon m_IntakeBottom;

        int m_PeakCurrentLimit = -1; //FIX
        int m_PeakCurrentDuration = -1; //FIX
        int m_ContinousCurrentLimit = -1; //FIX


};

