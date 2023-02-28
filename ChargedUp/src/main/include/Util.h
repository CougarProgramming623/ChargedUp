
#pragma once 

inline void DebugOutF(const std::string_view message) {
  frc2::CommandScheduler::GetInstance().Schedule(
      new frc2::PrintCommand(message));
}

#define BUTTON_L(id) \
  [&] { return Robot::GetRobot()->GetButtonBoard().GetRawButton(id); }
#include <frc2/command/PrintCommand.h>