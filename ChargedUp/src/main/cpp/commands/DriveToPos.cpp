#include "commands/DriveToPos.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"

DriveToPos::DriveToPos(frc::Pose2d targetPose, frc::Rotation2d targetAngle) {
    AddRequirements(&Robot::s_Instance->GetDriveTrain());
    m_TargetPose = targetPose;
    m_TargetAngle = targetAngle;
}

void DriveToPos::Initialize(){
    m_Current = Robot::s_Instance->GetDriveTrain().m_Odometry.GetPose();
    
}

void DriveToPos::Execute() {
    Robot* r = Robot::s_Instance; 
    m_Current = r->GetDriveTrain().m_Odometry.GetPose();   
    r->GetDriveTrain().BaseDrive(
        r->GetDriveTrain().m_DriveController.Calculate(m_Current, m_TargetPose, m_TargetAngle)
    );
}