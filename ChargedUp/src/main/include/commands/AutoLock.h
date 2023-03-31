#pragma once
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Rotation2d.h>


class AutoLock : public frc2::CommandHelper<frc2::CommandBase, AutoLock> {
 public:
  explicit AutoLock();
  //~DriveWithJoystick();

  void Initialize() override;
  void Execute() override;
  double Deadfix(double in, double deadband);

  frc::Rotation2d m_GoalTheta;
};