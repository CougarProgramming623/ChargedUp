#pragma once
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class PivotToPos : public frc2::CommandHelper<frc2::CommandBase, PivotToPos> {
 public:
  explicit PivotToPos(double angle);

  void Initialize() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
	double angleToGoTo;
  double ticksToMove;
  double degreesToMove;
};