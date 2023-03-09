#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/Joystick.h>
#include <frc/Servo.h>
#include <ctre/phoenix/motorcontrol/can/BaseMotorController.h>
#include <frc/Joystick.h>
#include <frc2/command/button/Button.h>
#include <frc/AnalogInput.h>


#include <math.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "Constants.h"
#include "Util.h"
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>

using ctre::phoenix::motorcontrol::can::TalonSRX;

class Arm2 : public frc2::SubsystemBase {

	public:

    Arm2();
	void Init();
	void SetButtons();


    private:

    frc2::Button m_EverybotIntakeButton;
    ctre::phoenix::motorcontrol::can::TalonSRX m_EverybotIntakeMotor;
};