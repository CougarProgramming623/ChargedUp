#include "commands/AutoBalance.h"
#include "Robot.h"

AutoBalance:: AutoBalance() {
    AddRequirements(&Robot::GetRobot()->GetDriveTrain());
}

void AutoBalance::Initialize() {
  
}

void AutoBalance::Execute() {

    double kP = 0;
    double kI = 0;

    double m_currentAngleX = Robot::GetRobot()->GetNavX().GetPitch();
    double m_currentAngleY = Robot::GetRobot()->GetNavX().GetYaw();
    double m_currentAngleT = Robot::GetRobot()->GetNavX().GetRoll();

    double errorX = 0 /*setpoint constant*/ - m_currentAngleX;
    double errorY = 0 /*setpoint constant*/ - m_currentAngleY;
    double errorT = 0 /*setpoint constant*/ - m_currentAngleT;

    double outputX = *(Robot::GetRobot()->previousValueX) + (kP * errorX) + (kI * (*(Robot::GetRobot()->previousErrorX)));
    double outputY = *(Robot::GetRobot()->previousValueY) + (kP * errorY) + (kI * (*(Robot::GetRobot()->previousErrorY)));
    double outputT = *(Robot::GetRobot()->previousValueT) + (kP * errorT) + (kI * (*(Robot::GetRobot()->previousErrorT)));

    
    *(Robot::GetRobot()->previousErrorX) += errorX;
    *(Robot::GetRobot()->previousValueX) = outputX;
    *(Robot::GetRobot()->previousErrorY) += errorY;
    *(Robot::GetRobot()->previousValueY) = outputY;
    *(Robot::GetRobot()->previousErrorT) += errorT;
    *(Robot::GetRobot()->previousValueT) = outputT;

    //double output = Y[k-1] + kP * U[k] + kI * U[k-1]  
    //  current value = previous value + kP * currentError + kI * previousError

    Robot *r = Robot::GetRobot();
    
    r->GetDriveTrain().BaseDrive(
        frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            units::meters_per_second_t(outputX * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND), //x
            units::meters_per_second_t(outputY * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND), //y
            units::radians_per_second_t(outputT * r->GetDriveTrain().kMAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND), //rotation
            frc::Rotation2d(units::radian_t(Deg2Rad(-fmod(360 + 90 - r->GetNavX().GetAngle(), 360))))
        )
    );
}