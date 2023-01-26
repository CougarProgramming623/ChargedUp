#include "commands/DriveToPos.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"

DriveToPos::DriveToPos(frc::Trajectory trajectory, frc::Rotation2d targetAngle) {
    AddRequirements(&Robot::s_Instance->GetDriveTrain());
    m_Trajectory = trajectory;
    m_TargetAngle = targetAngle;
}

void DriveToPos::Initialize(){
    m_Current = Robot::s_Instance->GetDriveTrain().m_Odometry.GetPose();
}

void DriveToPos::Execute() {
    Robot* r = Robot::s_Instance; 
    m_Current = r->GetDriveTrain().m_Odometry.GetPose(); 
    const frc::Trajectory::State TargetPose = m_Trajectory.Sample(2_s); 
    r->GetDriveTrain().BaseDrive(
        r->GetDriveTrain().m_DriveController.Calculate(m_Current, TargetPose, m_TargetAngle)
    );
}