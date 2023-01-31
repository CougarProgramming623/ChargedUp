#include <frc2/command/PrintCommand.h>

#pragma once 

inline void DebugOutF(const std::string_view message) {
  frc2::CommandScheduler::GetInstance().Schedule(
      new frc2::PrintCommand(message));
}