#pragma once
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <AHRS.h>
#include <frc/Joystick.h>

#include <pathplanner/lib/PathPlanner.h>
#include "RobotContainer.h"
#include "subsystems/Arm.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
  static Robot* GetRobot() { return s_Instance; }


  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;


  

  static Robot* GetRobot() { return s_Instance; }

  inline frc::Joystick& GetJoyStick() { return m_Joystick; }

  static Robot* s_Instance;


 private:
  static Robot* s_Instance;
  std::optional<frc2::CommandPtr> m_autonomousCommand;


  frc::Joystick m_Joystick = frc::Joystick(1);
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* m_autonomousCommand = nullptr;

  RobotContainer m_container;

  Arm m_Arm;
};
