#include "commands/TrajectoryCommand.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "Util.h"

using namespace pathplanner;


//Constructs a Trajectory Command based on a PathPlannerTrajectory
TrajectoryCommand::TrajectoryCommand(PathPlannerTrajectory trajectory) {
    AddRequirements(&Robot::GetRobot()->GetDriveTrain());
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
    Robot* r = Robot::GetRobot();
    
    frc::ChassisSpeeds speeds = r->GetDriveTrain().GetHolonomicController().Calculate(r->GetDriveTrain().GetOdometry()->GetPose(), m_Trajectory.sample(m_Timer.Get()).asWPILibState(), /*frc::Rotation2d(units::radian_t(Deg2Rad(90)))*/m_Trajectory.sample(m_Timer.Get()).holonomicRotation);

    speeds.vy = -speeds.vy;
    speeds.omega = -speeds.omega;

    r->GetDriveTrain().BaseDrive(
        speeds
    );
    //DebugOutF(std::to_string(m_Trajectory.sample(m_Timer.Get()).holonomicRotation.Degrees().value()));
}

void TrajectoryCommand::End(bool interrupted){
    DebugOutF("Ending follow");
}

//End command when close to intended pose
bool TrajectoryCommand::IsFinished(){
    return false;
}