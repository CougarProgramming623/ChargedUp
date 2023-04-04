#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/AnalogInput.h>
#include "Constants.h"


class WristToPosAuto : public frc2::CommandHelper<frc2::CommandBase, WristToPosAuto> {
	public:
		explicit WristToPosAuto(double deg);
		void Initialize() override;
		void Execute() override;
  		void End(bool interrupted) override;
		bool IsFinished() override;

		double targetDegrees;
		double startingDegrees;
		double ticksToMove;
};