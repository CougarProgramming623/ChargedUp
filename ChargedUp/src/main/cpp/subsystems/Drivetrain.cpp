#include "subsystems/Drivetrain.h"
#include "Robot.h"

#include <frc/trajectory/Trajectory.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/Timer.h>

//Constructor
DriveTrain::DriveTrain()
    : m_FrontLeftLocation(units::meter_t (DRIVETRAIN_TRACKWIDTH_METERS / 2.0), units::meter_t (-DRIVETRAIN_WHEELBASE_METERS / 2.0)),
      m_FrontRightLocation(units::meter_t (DRIVETRAIN_TRACKWIDTH_METERS / 2.0), units::meter_t (DRIVETRAIN_WHEELBASE_METERS / 2.0)),
      m_BackLeftLocation(units::meter_t (-DRIVETRAIN_TRACKWIDTH_METERS / 2.0), units::meter_t (-DRIVETRAIN_WHEELBASE_METERS / 2.0)),
      m_BackRightLocation(units::meter_t (-DRIVETRAIN_TRACKWIDTH_METERS / 2.0), units::meter_t (DRIVETRAIN_WHEELBASE_METERS / 2.0)),
      m_Kinematics(m_FrontLeftLocation, m_FrontRightLocation, m_BackLeftLocation, m_BackRightLocation),
      m_Rotation(0_rad),
      m_ModulePositions( wpi::array<frc::SwerveModulePosition, 4>
         (m_FrontLeftModule.GetPosition(), m_FrontRightModule.GetPosition(), m_BackLeftModule.GetPosition(), m_BackRightModule.GetPosition())),
      //m_Odometry(m_Kinematics, m_Rotation, m_ModulePositions, frc::Pose2d(0_m, 0_m, 0_rad)),
      m_Odometry(m_Kinematics, m_Rotation, ( wpi::array<frc::SwerveModulePosition, 4>
         (m_FrontLeftModule.GetPosition(), m_FrontRightModule.GetPosition(), m_BackLeftModule.GetPosition(), m_BackRightModule.GetPosition())), frc::Pose2d(0_m, 0_m, 0_rad)),
      m_FrontLeftModule(FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_ENCODER_PORT, -137),
      m_FrontRightModule(FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_ENCODER_PORT, -287),
      m_BackLeftModule(BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_ENCODER_PORT, -140.6),
      m_BackRightModule(BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_ENCODER_PORT, -2),
      m_ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s}, 
      m_xController(.7, 0.4, 0.2),
      m_yController(.7, 0.4, 0.2),
      m_ThetaController(15.5, 25, 0.01, frc::TrapezoidProfile<units::radian>::Constraints{3.14_rad_per_s, (1/2) * 3.14_rad_per_s / 1_s}),
      m_HolonomicController(m_xController, m_yController, m_ThetaController)
{}

/*
Is called periodically
Passes module states to motors and updates odometry
*/
void DriveTrain::Periodic(){

  if((m_ModuleStates[0].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE) == 0 && ((double) m_ModuleStates[0].angle.Radians() == 0)){
    m_FrontLeftModule.m_SteerController.motor.Set(ControlMode::PercentOutput, 0);
    m_FrontLeftModule.m_DriveController.motor.Set(ControlMode::PercentOutput, 0);
  } else {
    m_FrontLeftModule.Set(m_ModuleStates[0].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE, (double) m_ModuleStates[0].angle.Radians());
  }

  if((m_ModuleStates[1].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE == 0) && ((double) m_ModuleStates[1].angle.Radians() == 0)){
    m_FrontRightModule.m_SteerController.motor.Set(ControlMode::PercentOutput, 0);
    m_FrontRightModule.m_DriveController.motor.Set(ControlMode::PercentOutput, 0);
  } else {
    m_FrontRightModule.Set(m_ModuleStates[1].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE, (double) m_ModuleStates[1].angle.Radians());
  }

  if((m_ModuleStates[2].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE == 0) && ((double) m_ModuleStates[2].angle.Radians() == 0)){
    m_BackLeftModule.m_SteerController.motor.Set(ControlMode::PercentOutput, 0);
    m_BackLeftModule.m_DriveController.motor.Set(ControlMode::PercentOutput, 0);
  } else {
    m_BackLeftModule.Set(m_ModuleStates[2].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE, (double) m_ModuleStates[2].angle.Radians());
  }

  if((m_ModuleStates[3].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE == 0) && ((double) m_ModuleStates[3].angle.Radians() == 0)){
    m_BackRightModule.m_SteerController.motor.Set(ControlMode::PercentOutput, 0);
    m_BackRightModule.m_DriveController.motor.Set(ControlMode::PercentOutput, 0);
  } else {
    m_BackRightModule.Set(m_ModuleStates[3].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE, (double) m_ModuleStates[3].angle.Radians());
  }

  m_Rotation = frc::Rotation2d(units::radian_t(Deg2Rad(Robot::GetRobot()->GetAngle())));

  //m_Rotation = frc::Rotation2d(frc::Rotation2d(units::radian_t(Deg2Rad(-fmod(360 - 180 + 90 - Robot::s_Instance->GetNavX().GetAngle(), 360)))).Cos(), -frc::Rotation2d(units::radian_t(Deg2Rad(-fmod(360 - 180 + 90 - Robot::s_Instance->GetNavX().GetAngle(), 360)))).Sin());

  m_ModulePositions = wpi::array<frc::SwerveModulePosition, 4>(m_FrontLeftModule.GetPosition(), m_FrontRightModule.GetPosition(), m_BackLeftModule.GetPosition(), m_BackRightModule.GetPosition());

  m_Odometry.Update(m_Rotation, m_ModulePositions);
    
  DebugOutF("X: " + std::to_string(m_Odometry.GetPose().X().value()));
  DebugOutF("Y: " + std::to_string(m_Odometry.GetPose().Y().value()));
  DebugOutF("Deg: " + std::to_string(m_Odometry.GetPose().Rotation().Degrees().value()));

}


//Converts chassis speed object and updates module states
void DriveTrain::BaseDrive(frc::ChassisSpeeds chassisSpeeds){
  m_ChassisSpeeds = chassisSpeeds;
  // DebugOutF("Y speed: " + std::to_string(Rad2Deg(m_ChassisSpeeds.vy.value())));
  // DebugOutF("Omega: " + std::to_string(Rad2Deg(m_ChassisSpeeds.omega.value())));
  auto [fl, fr, bl, br] = m_Kinematics.ToSwerveModuleStates(m_ChassisSpeeds);
  m_ModuleStates = {fl, fr, bl, br};
}

//Initializes rotation angle and default command
void DriveTrain::DriveInit(){
  m_Rotation = frc::Rotation2d(units::radian_t(Robot::GetRobot()->GetNavX().GetAngle()));
  SetDefaultCommand(DriveWithJoystick());
  //m_ThetaController.EnableContinuousInput(-M_PI, M_PI);
}

//Sets breakmode
void DriveTrain::BreakMode(bool on){
  m_FrontLeftModule.BreakMode(on);
  m_FrontRightModule.BreakMode(on);
  m_BackLeftModule.BreakMode(on);
  m_BackRightModule.BreakMode(on);
}

// void DriveTrain::TrajectoryDrive(std::array<frc::SwerveModuleState, 4> states){
//   m_ModuleStates = states;
// } 

// void DriveTrain::TrajectoryFollow(frc::Trajectory trajectory, std::function<frc::Rotation2d()> Rotation){
//     frc2::CommandScheduler::GetInstance().Schedule(new frc2::SwerveControllerCommand<4>{
//       trajectory, 
//       m_Odometry.GetPose(),
//       m_Kinematics,
//       m_xController,
//       m_yController,
//       m_ThetaController,
//       Rotation,
//       TrajectoryDrive(m_ModuleStates),
//       *this
//   });
// }

// void DriveTrain::TrajectoryFollow(frc::Trajectory trajectory){
//   frc2::CommandScheduler::GetInstance().Schedule(new frc2::SwerveControllerCommand<4>{
//     trajectory, 
//     [this]() { return m_Odometry.GetPose(); }, 
//     m_Kinematics, 
//     m_xController, 
//     m_yController, 
//     m_ThetaController, 
//     //[&](){ return trajectory.States()},
//     [this](auto moduleStates) { TrajectoryDrive(moduleStates); }, 
//     {&Robot::s_Instance->GetDriveTrain()}
//   });
// };