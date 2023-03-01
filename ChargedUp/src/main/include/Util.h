#include <frc2/command/PrintCommand.h>

#pragma once 

inline void DebugOutF(const std::string_view message) {
  frc2::CommandScheduler::GetInstance().Schedule(
      new frc2::PrintCommand(message));
}

#define BUTTON_L(id) \
  [&] { return Robot::GetRobot()->GetButtonBoard().GetRawButton(id); }

#define OVERRIDE_BUTTON_L(ID) \
  [&] { return Robot::GetRobot()->GetButtonBoard().GetRawButton(ID) && !Robot::GetRobot()->GetButtonBoard().GetRawButton(ARM_OVERRIDE); }