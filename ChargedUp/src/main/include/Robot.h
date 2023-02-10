#pragma once
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include "RobotContainer.h"
#include "subsystems/DriveTrain.h"
#include <AHRS.h>
#include <frc/Joystick.h>

class Robot : public frc::TimedRobot {
 public:

  Robot();

  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  //void SimulationInit() override;
  //void SimulationPeriodic() override;

  inline AHRS& GetNavX() { return m_NavX; }
  inline void zeroGyroscope() {m_NavX.ZeroYaw();}
  inline double getYaw() {return m_NavX.GetYaw();}
  inline double getPitch() {return m_NavX.GetPitch();}


  static Robot* GetRobot() { return s_Instance; } 
  inline DriveTrain& GetDriveTrain() { return m_DriveTrain; }
  inline frc::Joystick& GetJoyStick() { return m_Joystick; }

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

  RobotContainer m_container;

  DriveTrain m_DriveTrain;
};
