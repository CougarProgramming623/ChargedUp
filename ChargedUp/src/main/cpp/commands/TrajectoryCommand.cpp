#include "commands/TrajectoryCommand.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "Util.h"

using namespace pathplanner;


//Constructs a Trajectory Command based on a PathPlannerTrajectory
TrajectoryCommand::TrajectoryCommand(PathPlannerTrajectory trajectory) {
    AddRequirements(&Robot::s_Instance->GetDriveTrain());
    m_Trajectory = trajectory;
}

//Start timer
void TrajectoryCommand::Initialize(){
    m_Timer.Start();
}

/*
Calculates required chassis speed to match trajactory
Passes ChassisSpeed object to BaseDrive() function
*/
void TrajectoryCommand::Execute() {
    Robot* r = Robot::s_Instance;
    
    r->GetDriveTrain().BaseDrive(
        r->GetDriveTrain().GetHolonomicController().Calculate(r->GetDriveTrain().GetOdometry()->GetPose(), m_Trajectory.sample(m_Timer.Get()).pose, m_Trajectory.sample(m_Timer.Get()).velocity(), m_Trajectory.sample(m_Timer.Get()).holonomicRotation)
    );   
}

void TrajectoryCommand::End(bool interrupted){
    DebugOutF("Ending follow");
}

//End command when close to intended pose
bool TrajectoryCommand::IsFinished(){
    return (
        m_Trajectory.getEndState().asWPILibState().pose.X() > Robot::s_Instance->GetDriveTrain().GetOdometry()->GetPose().X() - .01_m &&
        m_Trajectory.getEndState().asWPILibState().pose.X() < Robot::s_Instance->GetDriveTrain().GetOdometry()->GetPose().X() + .01_m
    ) && (
        m_Trajectory.getEndState().asWPILibState().pose.Y() > Robot::s_Instance->GetDriveTrain().GetOdometry()->GetPose().Y() - .01_m &&
        m_Trajectory.getEndState().asWPILibState().pose.Y() < Robot::s_Instance->GetDriveTrain().GetOdometry()->GetPose().Y() + .01_m
    // ) && (
    //     m_Trajectory.getEndState().asWPILibState().pose.Rotation().Degrees() > Robot::s_Instance->GetDriveTrain().GetOdometry()->GetPose().Y() - .01_m && TODO
    //     m_Trajectory.getEndState().asWPILibState().pose.Y() < Robot::s_Instance->GetDriveTrain().GetOdometry()->GetPose().Y() + .01_m    
    );
}