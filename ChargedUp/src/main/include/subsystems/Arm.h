#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/Joystick.h>
#include <frc/Servo.h>
#include <ctre/phoenix/motorcontrol/can/BaseMotorController.h>
#include <frc/Joystick.h>
#include <frc2/command/button/Button.h>
#include <frc/AnalogInput.h>
#include <math.h>
#include <ctre/phoenix/sensors/CANCoder.h>

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
#include <frc2/command/ParallelCommandGroup.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "./commands/PivotToPos.h"
#include "./commands/DynamicIntake.h"



using ctre::phoenix::motorcontrol::can::TalonFX;
using ctre::phoenix::motorcontrol::can::TalonSRX;


class Arm : public frc2::SubsystemBase {

	public:

	Arm();
	void Init();
	void SetButtons();
	void WristInit();
	

	frc2::FunctionalCommand* ManualControls();
	
	inline TalonSRX& GetPivotMotor() {return m_Pivot;}
	inline TalonSRX& GetWristMotor() {return m_Wrist;} 
	// inline TalonSRX& GetTopIntakeMotor() {return m_TopIntake;}
	inline TalonSRX& GetBottomIntakeMotor() {return m_BottomIntake;}
	inline ctre::phoenix::sensors::CANCoder& GetPivotCANCoder() {return m_PivotCANCoder;}

	inline frc2::Button& GetCubeModeButton() {return m_CubeMode; }
	inline frc2::Button& GetConeModeButton() {return m_ConeMode; }
	inline frc2::Button& GetIntakeButton() {return m_IntakeButton; }
	inline frc2::Button& GetOuttakeButton() {return m_OuttakeButton; }

	private:
	
	//motors
	TalonSRX m_Pivot; 
	ctre::phoenix::sensors::CANCoder m_PivotCANCoder{PIVOT_CAN_ID};
	TalonSRX m_Wrist; 
	// TalonSRX m_TopIntake;
	TalonSRX m_BottomIntake;

	//buttons
	frc2::Button m_TransitMode;
	frc2::Button m_GroundPickupMode;
	frc2::Button m_PlacingMode;

	frc2::Button m_Override;
	frc2::Button m_Override2;

	frc2::Button m_ConeMode;
	frc2::Button m_CubeMode;

	frc2::Button m_IntakeButton;
	frc2::Button m_OuttakeButton;

	frc::Timer m_Timer;

	frc2::SequentialCommandGroup* m_Top;
	frc2::SequentialCommandGroup* m_Mid;
	frc2::SequentialCommandGroup* m_Bot;
};