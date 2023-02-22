#include "subsystems/Arm.h"

using ctre::phoenix::motorcontrol::ControlMode;
using ctre::phoenix::motorcontrol::can::TalonFX;

Arm::Arm() :
	m_Pivot(PIVOT_MOTOR),
	m_Extraction(EXTRACTION_MOTOR),
	m_LeftBrake(LEFT_BRAKE),
	// m_RightBrake(RIGHT_BRAKE),
	m_UnlockPivot([&] {return m_ButtonBoard.GetRawButton(TELE_NUKE);}),
	m_BrakeButton([&] {return m_ButtonBoard.GetRawButton(RELEASE_BUTTON);}),
	m_TestingPOTButton([&]{return m_ButtonBoard.GetRawButton(FEED_BUTTON);})
	{}

void Arm::Init() {
	// m_LeftBrake.Set(0);
	// m_RightBrake.Set(0);
	m_Pivot.SetSelectedSensorPosition(0);
	SetButtons();

	m_Pivot.ConfigAllowableClosedloopError(0.0, 0.0, 0.0);
	m_Pivot.Config_kP(0.0, 0, 0.0);
	m_Pivot.Config_kI(0.0, 0, 0.0);
	m_Pivot.Config_kD(0.0, 0, 0.0);
	m_Pivot.Config_kF(0.0, 0, 0.0);
	m_Pivot.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

}

void Arm::PrintPosition() {
	DebugOutF(std::to_string(m_Pivot.GetSelectedSensorPosition()));
}

void Arm::SetButtons() {
	m_BrakeButton.WhenPressed(Telescope(2)); //"release"8 button
}


frc2::FunctionalCommand Arm::PivotToPosition(double angleSetpoint) {
	angle = angleSetpoint;
	return frc2::FunctionalCommand(
          [&] {  // onInit
			startingTicks = m_Pivot.GetSelectedSensorPosition();
			double currentAngle = PivotTicksToDeg(m_Pivot.GetSelectedSensorPosition()); //current angle of the arm
			double degToMove = angle - currentAngle; //how many degrees the arm needs to move in the correct direction
			ticksToMove = PivotDegToTicks(degToMove); //how many ticks the pivot motor needs to move in the correct direction
			setpoint = startingTicks + ticksToMove;
          },[&] {  // onExecute		  	
			m_Pivot.Set(ControlMode::Position, setpoint);
          }, [&](bool e) {  // onEnd
			m_Pivot.Set(ControlMode::PercentOutput, 0);
		  }, [&] {  // isFinished
			return (abs(m_Pivot.GetSelectedSensorPosition() - setpoint) < 200); //deadband of 100 ticks 
		  });
}

// Toggles the status of the brakes; currently assuming 0 is off, 1 is on
void Arm::ToggleBrakes(bool isBraked) {
	if (isBraked) {
		m_LeftBrake.Set(1);
		// m_RightBrake.Set(1);	
	} else { 
		m_LeftBrake.Set(0);
		// m_RightBrake.Set(0);
	}
}

//length should be a setpoint in inches || assumes forward power is increase length
frc2::FunctionalCommand Arm::Telescope(double setpoint) {
	setpointLength = setpoint;
	return frc2::FunctionalCommand(
		[&] {  // onInit
		DebugOutF(std::to_string(setpointLength));
		},[&] {  // onExecute		  	
		armLength = StringPotUnitsToInches(m_StringPot.GetValue());

		if(armLength < setpointLength) m_Extraction.Set(ControlMode::PercentOutput, .5);
		else if(armLength > setpointLength) m_Extraction.Set(ControlMode::PercentOutput, -.5);

		DebugOutF(std::to_string(armLength));
		DebugOutF(std::to_string(m_StringPot.GetValue()));
		}, [&](bool e) {  // onEnd
		m_Extraction.Set(ControlMode::PercentOutput, 0);
		}, [&] {  // isFinished
			return (abs(armLength - setpointLength) < .01);
		});
}



//------------------------------------------------------------------------------------------------------------------- constant might be used later dont delete
// 	//angles use parallel to ground on side of arm as zero
// 	double coneAngles[] = {49.74, 44.83}; // {low target angle, high target angle}
// 	double cubeAngles[] = {30.14, 25.90}; // {low target angle, high target angle}

// 	double coneArmLengths[] = {40.62, 60.99}; //same as cone arrays (in inches)
// 	double cubeArmLengths[] = {40.83, 74.41}; // ^^^

// void Arm::LoadingReady() {
// 	Telescope(38.19);
// 	PivotToPosition(103.43);
// }