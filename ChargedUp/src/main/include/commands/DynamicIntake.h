#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class DynamicIntake : public frc2::CommandHelper<frc2::CommandBase, DynamicIntake> {
	public:
		explicit DynamicIntake();
		void Initialize() override;
		void Execute() override;
  		void End(bool interrupted) override;
		bool IsFinished() override;
};