#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class WristToPos : public frc2::CommandHelper<frc2::CommandBase, WristToPos> {
	public:
		explicit WristToPos(double degPos);
		void Initialize() override;
		void Execute() override;
  		void End(bool interrupted) override;
		bool IsFinished() override;

		double targetDegree;
};