#pragma once
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>


class AutoLock : public frc2::CommandHelper<frc2::CommandBase, AutoLock> {
 public:
  explicit AutoLock();
  //~DriveWithJoystick();

  void Initialize() override;
  void Execute() override;
  double Deadfix(double in, double deadband);

  int m_GoalTheta;
};