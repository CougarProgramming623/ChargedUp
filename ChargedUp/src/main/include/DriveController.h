#pragma once

#include <ctre/phoenix/motorcontrol/NeutralMode.h>
//#include <ctre/phoenix/motorcontrol/StatusFrame>
//#include <ctre/phoenix/motorcontrol/TalonFXControlMode.h>
//#include <ctre/phoenix/motorcontrol/TalonFXInvertType.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
//#include <ctre/phoenix/motorcontrol/can/TalonFXConfiguration.h>

using ctre::phoenix::motorcontrol::can::TalonFX;

class DriveController {
    public:
        DriveController(int ID);

        void SetReferenceVoltage(double voltage);
        void GetReferenceVoltage(double voltage);

        double GetStateVelocity();

        void BreakMode(bool on);
    private:
        TalonFX motor;
        double nominalVoltage = 12;  //FIX it is double.NaN in the java and i still dont know what that means
        double currentLimit;    //FIX it is double.NaN in the java and i still dont know what that means


};  