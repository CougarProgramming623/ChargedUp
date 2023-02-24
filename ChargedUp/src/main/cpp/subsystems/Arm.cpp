#include "subsystems/Arm.h"

using ctre::phoenix::motorcontrol::ControlMode;
using ctre::phoenix::motorcontrol::can::TalonFX;

Arm::Arm() :
	m_Pivot(PIVOT_MOTOR),
	m_Extraction(EXTRACTION_MOTOR),
	m_LeftBrake(LEFT_BRAKE),
	// m_RightBrake(RIGHT_BRAKE),
	m_TestJoystickButton([&] {return m_Joystick.GetRawButton(1);})
	{}

void Arm::Init() {
	// m_LeftBrake.Set(0);
	// m_RightBrake.Set(0);
	m_Pivot.SetSelectedSensorPosition(0);
	SetButtons();
	ToggleBrakes(true);
	m_Pivot.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	m_Extraction.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

}



void Arm::SetButtons() {
	m_TestJoystickButton.WhenPressed(Telescope(2));
}


frc2::FunctionalCommand Arm::PivotToPosition(double angleSetpoint) {
	Angle = angleSetpoint;
	return frc2::FunctionalCommand(
          [&] {  // onInit
			StartingTicks = m_Pivot.GetSelectedSensorPosition();
			double currentAngle = PivotTicksToDeg(m_Pivot.GetSelectedSensorPosition()); //current angle of the arm
			double degToMove = Angle - currentAngle; //how many degrees the arm needs to move in the correct direction
			TicksToMove = PivotDegToTicks(degToMove); //how many ticks the pivot motor needs to move in the correct direction
			Setpoint = StartingTicks + TicksToMove;
          },[&] {  // onExecute		  	
			m_Pivot.Set(ControlMode::Position, Setpoint);
          }, [&](bool e) {  // onEnd
			m_Pivot.Set(ControlMode::PercentOutput, 0);
		  }, [&] {  // isFinished
			return (abs(m_Pivot.GetSelectedSensorPosition() - Setpoint) < 200); //deadband of 100 ticks 
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
	m_brakesActive = isBraked;
}

//length should be a setpoint in inches || assumes forward power is increase length
frc2::FunctionalCommand Arm::Telescope(double Setpoint) {
	SetpointLength = Setpoint;
	return frc2::FunctionalCommand(
		[&] {  // onInit
		ToggleBrakes(false);
		DebugOutF(std::to_string(SetpointLength));
		},[&] {  // onExecute		  	
		ArmLength = StringPotUnitsToInches(m_StringPot.GetValue());

		if(ArmLength < SetpointLength) m_Extraction.Set(ControlMode::PercentOutput, .5);
		else if(ArmLength > SetpointLength) m_Extraction.Set(ControlMode::PercentOutput, -.5);

		DebugOutF(std::to_string(ArmLength));
		DebugOutF(std::to_string(m_StringPot.GetValue()));
		}, [&](bool e) {  // onEnd
			ToggleBrakes(true);
			m_Extraction.Set(ControlMode::PercentOutput, 0);
		}, [&] {  // isFinished
			return (abs(ArmLength - SetpointLength) < .01);
		});
}

frc2::FunctionalCommand Arm::Squeeze(bool shouldSqueeze) {	
	if(shouldSqueeze) {
		return frc2::FunctionalCommand( [&] { //onInit
			TicksToUndoSqueeze = m_Extraction.GetSelectedSensorPosition();
			m_Extraction.Set(ControlMode::PercentOutput, 1);
		}, [&] {//onExecute
			//empty
		}, [&](bool e) {//onEnd
			m_Extraction.Set(ControlMode::PercentOutput, 0);
			TicksToUndoSqueeze = TicksToUndoSqueeze - m_Extraction.GetSelectedSensorPosition();
		}, [&] {//isFinished
			return m_Extraction.GetSupplyCurrent() < SQUEEZE_AMP_THRESHHOLD;
		});}
	else {
		return frc2::FunctionalCommand( [&] { //onInit
			TicksToUndoSqueeze = m_Extraction.GetSelectedSensorPosition() - TicksToUndoSqueeze;
			m_Extraction.Set(ControlMode::Position, TicksToUndoSqueeze);
		}, [&] {//onExecute
			//empty
		}, [&](bool e) {//onEnd
			m_Extraction.Set(ControlMode::PercentOutput, 0);
		}, [&] {//isFinished
			return m_Extraction.GetSelectedSensorPosition() - TicksToUndoSqueeze  < 100; 
		});
		}
	}