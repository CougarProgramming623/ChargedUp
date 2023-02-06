#include "commands/TrajectoryCommand.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"

using namespace pathplanner;

TrajectoryCommand::TrajectoryCommand(PathPlannerTrajectory trajectory) {
    AddRequirements(&Robot::s_Instance->GetDriveTrain());
    m_Trajectory = trajectory;
}

void TrajectoryCommand::Initialize(){
    m_Timer.Start();
}

void TrajectoryCommand::Execute() {
    Robot* r = Robot::s_Instance;
    
    r->GetDriveTrain().BaseDrive(
        r->GetDriveTrain().m_HolonomicController.Calculate(r->GetDriveTrain().m_Odometry.GetPose(), m_Trajectory.sample(m_Timer.Get()).asWPILibState(), m_Trajectory.sample(m_Timer.Get()).holonomicRotation)
    );   
}

void TrajectoryCommand::End(bool interrupted){
    
}

bool TrajectoryCommand::IsFinished(){
    return m_Trajectory.getEndState().asWPILibState().pose == Robot::s_Instance->GetDriveTrain().m_Odometry.GetPose();
}