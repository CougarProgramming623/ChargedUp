#include "subsystems/Arm2.h"
#include "Robot.h"
#include "frc2/command/PrintCommand.h"
#include <frc/DriverStation.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>

using ctre::phoenix::motorcontrol::ControlMode;
using ctre::phoenix::motorcontrol::can::TalonSRX;

Arm2::Arm2() 
: 
m_EverybotIntakeButton(BUTTON_L_TWO(6))
{}


void Arm2::Init(){}

void Arm2::SetButtons(){

    m_EverybotIntake.WhenHeld(frc2::InstantCommand([&]{
        DebugOutF("intake button yay");

        m_EverybotIntake.Set(ControlMode::PercentOutput, 1);
    }))

}

