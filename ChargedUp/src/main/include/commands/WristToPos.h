#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/AnalogInput.h>
#include "Constants.h"


class WristToPos : public frc2::CommandHelper<frc2::CommandBase, WristToPos> {
	public:
		explicit WristToPos(double degPos);
		void Initialize() override;
		void Execute() override;
  		void End(bool interrupted) override;
		bool IsFinished() override;

		inline double WristDegToTicks(double degree) {return degree * WRIST_TICKS_PER_ARM_DEGREE;} //converts degrees to ticks of Pivot motor
		inline double WristTicksToDeg(double ticks) {ticks / WRIST_TICKS_PER_ARM_DEGREE;} //converts ticks to degrees of arm rotation

		double targetDegree;
};