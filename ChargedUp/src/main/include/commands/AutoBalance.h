#pragma once

#include <AHRS.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>
#include "Robot.h"

class AutoBalance
    :public frc2::CommandHelper<frc2::CommandBase, AutoBalance> {
    
public:
  explicit AutoBalance();

  void Initialize() override;
  void Execute() override;

 private:
  double m_currentAngle;
  double m_prevError;

};