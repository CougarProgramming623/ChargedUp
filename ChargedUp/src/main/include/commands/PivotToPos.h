#pragma once
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Constants.h"

class PivotToPos : public frc2::CommandHelper<frc2::CommandBase, PivotToPos> {
 public:
  explicit PivotToPos(double angle);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
  inline double PivotDegToTicks(double degree) {return degree * PIVOT_TICKS_PER_ARM_DEGREE;} //converts degrees to ticks of Pivot motor
	inline double PivotTicksToDeg(double ticks) {return ticks / PIVOT_TICKS_PER_ARM_DEGREE;} //converts ticks to degrees of arm rotation

 private:
	double angleToGoTo;
  double ticksToMove;
  double degreesToMove;
};