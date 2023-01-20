#include "SwerveModule.h"
#include "Util.h"
#include "Constants.h"

SwerveModule::SwerveModule(int driveID, int steerID, int encoderPort, double angleOffset):
    m_DriveController(driveID), 
    m_SteerController(steerID, encoderPort, angleOffset)
{}

double SwerveModule::GetDriveVelocity(){
    return m_DriveController.GetStateVelocity();
}

double SwerveModule::GetSteerAngle(){
    return m_SteerController.GetStateAngle();
}

void SwerveModule::BreakMode(bool on){
    m_DriveController.BreakMode(on);
}

void SwerveModule::Set(double driveVoltage, double steerAngle){
    steerAngle  = fmod(steerAngle, 2.0 * M_PI);
    if(steerAngle < 0.0){
        steerAngle += (2.0 * M_PI);
    }

    double difference = steerAngle - GetSteerAngle();

    if(difference >= M_PI) {
        steerAngle -= (2.0 * M_PI);
    }
    else if(difference < -M_PI) {
        steerAngle += (2.0 * M_PI);
    }

    difference = steerAngle - GetSteerAngle(); //recalc difference

    if(difference > M_PI / 2.0 || difference < (-M_PI) / 2.0) {
        steerAngle += M_PI;
        driveVoltage *= -1.0;
    }

    steerAngle = fmod(steerAngle, (2.0 * M_PI));
    if(steerAngle < 0.0) {
        steerAngle += (2.0 * M_PI);
    }

    m_SteerController.SetReferenceAngle(steerAngle);
    m_DriveController.SetReferenceVoltage(driveVoltage);
}