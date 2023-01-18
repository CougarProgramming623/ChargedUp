#pragma once

#include "DriveController.h"
#include "SteerController.h"


class SwerveModule {
    public:
        SwerveModule(int driveID, int steerID, int encoderPort, double angleOffset);

        double GetDriveVelocity();

        double GetSteerAngle();

        void Set(double driveVoltage, double steerAngle);
        void BreakMode(bool on);
    
        DriveController m_DriveController;
        SteerController m_SteerController;
};
