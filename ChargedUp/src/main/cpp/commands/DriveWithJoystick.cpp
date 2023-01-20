#include "commands/DriveWithJoystick.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"

DriveWithJoystick::DriveWithJoystick() {
    AddRequirements(&Robot::GetRobot()->GetDriveTrain());
}

//DriveWithJoystick::~DriveWithJoystick(){}

void DriveWithJoystick::Initialize(){

}

double DriveWithJoystick::Deadfix(double in, double deadband) {
    if(abs(in) < deadband) {
        return 0;
    }
    return in;
}



void DriveWithJoystick::Execute() {
    Robot* r = Robot::GetRobot();
    DebugOutF(std::to_string(fmod(360 + 90 - r->GetNavX().GetAngle(), 360)));
    
    r->GetDriveTrain().BaseDrive(
        frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            units::meters_per_second_t(Deadfix(r->GetJoyStick().GetRawAxis(1), 0.2) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND),
            units::meters_per_second_t(-Deadfix(r->GetJoyStick().GetRawAxis(0), 0.2) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND),
            units::radians_per_second_t(Deadfix(r->GetJoyStick().GetRawAxis(2), 0.2) * r->GetDriveTrain().kMAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
            frc::Rotation2d(units::radian_t(Deg2Rad(-fmod(360 + 90 - r->GetNavX().GetAngle(), 360))))
        )
    );


}