#pragma once
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>


class DriveWithJoystick : public frc2::CommandHelper<frc2::CommandBase, DriveWithJoystick> {
 public:
  explicit DriveWithJoystick();
  //~DriveWithJoystick();

  void Initialize() override;
  void Execute() override;
  double Deadfix(double in, double deadband);
};