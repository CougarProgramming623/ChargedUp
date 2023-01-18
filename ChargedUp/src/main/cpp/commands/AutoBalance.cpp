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

    double m_currentAngle = Robot::GetRobot()->GetNavX().GetPitch();

    double error = 0 /*setpoint constant*/ - m_currentAngle;

    double output = *(Robot::GetRobot()->previousValue) + (kP * error) + (kI * (*(Robot::GetRobot()->previousError)));

    
    *(Robot::GetRobot()->previousError) += error;
    *(Robot::GetRobot()->previousValue) = output;

    //double output = Y[k-1] + kP * U[k] + kI * U[k-1]  
    //  current value = previous value + kP * currentError + kI * previousError

    Robot *r = Robot::GetRobot();
    
    r->GetDriveTrain().BaseDrive(
        frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            units::meters_per_second_t(0 /*CHANGE*/ * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND), //x
            units::meters_per_second_t(-output * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND), //y
            units::radians_per_second_t(0 /*CHANGE*/ * r->GetDriveTrain().kMAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND), //rotation
            frc::Rotation2d(units::radian_t(Deg2Rad(-fmod(360 + 90 - r->GetNavX().GetAngle(), 360))))
        )
    );
}