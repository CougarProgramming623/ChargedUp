#include "Robot.h"

#include "subsystems/Arm.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Pose2d.h>
#include "Util.h"
#include "commands/TrajectoryCommand.h"
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/RobotController.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <ctre/phoenix/motorcontrol/can/BaseMotorController.h>
using ctre::phoenix::motorcontrol::ControlMode;

Arm::Arm()
    :   m_Wrist(WRIST_MOTOR),
        m_Pivot(PIVOT_MOTOR)
        // m_IntakeTop(INTAKE_TOP_MOTOR, ),
        // m_IntakeBottom(INTAKE_BOTTOM_MOTOR, )
    //stuff
{}

void Arm::ArmInit(){
    
    m_Pivot.SetSelectedSensorPosition(m_Pivot.GetSelectedSensorPosition() - Robot::GetRobot()->GetArm().m_OffsetTicks);
    m_Pivot.ConfigMotionCruiseVelocity(25000, 0);
    m_Pivot.ConfigMotionAcceleration(6000, 0);
    m_Pivot.Config_kF(0, 0.04716, 0);
    m_Pivot.Config_kP(0, 0.25, 0);
    m_Pivot.Config_kD(0, 0.05, 0);
}

int m_TicksToDeg(int ticks) {return (ticks / (2048 * 160)) * 360;}
int m_DegToTicks(int deg) {return (deg / 360) * 2048 * 160;}