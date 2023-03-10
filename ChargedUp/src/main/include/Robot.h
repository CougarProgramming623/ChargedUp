// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include <frc2/command/Command.h>

#include <pathplanner/lib/PathPlanner.h>
#include "subsystems/DriveTrain.h"
#include <AHRS.h>
#include <frc/Joystick.h>
#include <frc2/command/button/Button.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "COB.h"
#include "Vision.h"
#include "subsystems/Arm.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
  static inline Robot* GetRobot() { return s_Instance; }
  inline Arm& GetArm() { return m_Arm; }
  inline frc::Joystick& GetButtonBoard() { return m_ButtonBoard; }
  inline frc::Joystick& GetButtonBoardTwo() { return m_ButtonBoardTwo; }
  inline frc::Joystick& GetJoystick() { return m_Joystick; }

  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  inline AHRS& GetNavX() { return m_NavX; }
  inline void zeroGyroscope() {m_NavX.ZeroYaw();}
  inline double getYaw() {return m_NavX.GetYaw();}
  inline double getPitch() {return m_NavX.GetPitch();}

  inline DriveTrain& GetDriveTrain() { return m_DriveTrain; }
  inline frc::Joystick& GetJoyStick() { return m_Joystick; }

  double GetAngle() {return fmod(360 - GetNavX().GetYaw(), 360); }
  
  inline COB& GetCOB() { return m_COB; }
  inline Vision& GetVision() { return m_Vision; }

  double previousErrorX = 0;
  double previousErrorY = 0;
  double dErrorY = 0;
  double previousErrorT = 0;
  
  double previousValueX = 0;
  double previousValueY = 0;
  double previousValueT = 0;


 private:

  static Robot* s_Instance;

  AHRS m_NavX{frc::SPI::Port::kMXP};

  frc::Joystick m_Joystick = frc::Joystick(1);

  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* m_autonomousCommand = nullptr;

  Arm m_Arm;

  frc::Timer m_AutoTimer;
  DriveTrain m_DriveTrain;

  Vision m_Vision;

  COB m_COB;
  frc::Joystick m_ButtonBoard = frc::Joystick(0);
  frc::Joystick m_ButtonBoardTwo = frc::Joystick(2);
};
