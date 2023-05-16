#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>
#include <ctre/phoenix/motorcontrol/can/BaseMotorController.h>
#include "commands/PivotToPos.h"
#include "Constants.h"
#include "commands/DynamicIntake.h"
using ctre::phoenix::motorcontrol::can::BaseTalon;

DynamicIntake::DynamicIntake()
:
m_Intake(INTAKE_MOTOR),
m_Outtake(/*needs id*/)
//stuff
{}

void DynamicIntake::Intake(){


    if (m_IntakeTop.GetOutputCurrent() > 1000){ //FIX
        
    } 
}

void DynamicIntake::OutTake(){
    m_Outtake = 
}


double PivotToPos::GetPivot() {return m_PivotAngle;}