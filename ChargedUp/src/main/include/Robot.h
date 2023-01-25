#pragma once
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <AHRS.h>
#include <frc/Joystick.h>


#include "RobotContainer.h"
#include "subsystems/DriveTrain.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "COB.h"


class Robot : public frc::TimedRobot {
 public:
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
  inline double getYaw() {
    //return 360 m_NavX.GetYaw() - 90;
  }

  static Robot* GetRobot() { return s_Instance; }

  inline DriveTrain& GetDriveTrain() { return m_DriveTrain; }

  inline frc::Joystick& GetJoyStick() { return m_Joystick; }

  inline COB& GetCOB() { return m_COB; }

  static Robot* s_Instance;



 private:

  AHRS m_NavX{frc::SPI::Port::kMXP};

  frc::Joystick m_Joystick = frc::Joystick(1);

  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* m_autonomousCommand = nullptr;

  RobotContainer m_container;

  DriveTrain m_DriveTrain;

  COB m_COB;
};
