// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include "RobotContainer.h"
#include "subsystems/Arm.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
  static Robot* GetRobot() { return s_Instance; }
  inline frc::Joystick& GetButtonBoard() { return m_ButtonBoard; }
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
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  static Robot* s_Instance;
  std::optional<frc2::CommandPtr> m_autonomousCommand;

  RobotContainer m_container;

  Arm m_Arm;
  frc::Joystick m_ButtonBoard = frc::Joystick(0);
	frc::Joystick m_Joystick = frc::Joystick(1);
};
