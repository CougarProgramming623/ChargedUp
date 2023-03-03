#include "subsystems/Drivetrain.h"
#include "Robot.h"

#include <frc/trajectory/Trajectory.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/Timer.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>

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
      m_FrontLeftModule(FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_ENCODER_PORT, FRONT_LEFT_MODULE_STEER_OFFSET),
      m_FrontRightModule(FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_ENCODER_PORT, FRONT_RIGHT_MODULE_STEER_OFFSET),
      m_BackLeftModule(BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_ENCODER_PORT, BACK_LEFT_MODULE_STEER_OFFSET),
      m_BackRightModule(BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_ENCODER_PORT, BACK_RIGHT_MODULE_STEER_OFFSET),
      m_ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s}, 
      m_xController(.7, .4, 0.3),
      m_yController(.7, .4, 0.3),
      m_ThetaController(14, 25, 0.02, frc::TrapezoidProfile<units::radian>::Constraints{3.14_rad_per_s, (1/2) * 3.14_rad_per_s / 1_s}),
      m_HolonomicController(m_xController, m_yController, m_ThetaController),
      m_TestJoystickButton([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(1);}),
      m_Timer(),
      m_EventMap()
{
  m_EventMap.emplace(std::string("Mark 1"), std::make_shared<AutoBalance>(AutoBalance()));
  DebugOutF("Emplaced");
}

void DriveTrain::DriveInit(){
  m_Rotation = frc::Rotation2d(units::radian_t(Robot::GetRobot()->GetNavX().GetAngle()));
  SetDefaultCommand(DriveWithJoystick());
  m_TestJoystickButton.WhenPressed(new frc2::ParallelCommandGroup(
    DriveToPosCommand(),
    *Robot::GetRobot()->GetArm().PlaceElement(
      Robot::GetRobot()->GetArm().SelectedRow, 
      Robot::GetRobot()->GetArm().SelectedColumn
    )  
  ));

  m_Odometry.SetVisionMeasurementStdDevs(wpi::array<double, 3U> {0.15, 0.15, .261799});
  m_FrontLeftModule.m_DriveController.motor.SetInverted(true);
  //m_ThetaController.EnableContinuousInput(-M_PI, M_PI);
  //m_EventMap.emplace(std::string("Mark 1"), std::make_shared< frc2::PrintCommand>(std::string("Mark 1")));

}

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

  frc::Pose2d visionRelative = Robot::GetRobot()->GetVision().GetPoseBlue().RelativeTo(m_Odometry.GetEstimatedPosition());
  if(COB_GET_ENTRY(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).size() != 0){
    //DebugOutF("Second if");
    // DebugOutF("Relative X: " + std::to_string(
    // std::abs(visionRelative.X().value())));
    // DebugOutF("Y: " + std::to_string(
    // std::abs(visionRelative.Y().value())));
    // DebugOutF("Relative Z: " + std::to_string(
    // std::abs(-fmod(360 - visionRelative.Rotation().Degrees().value(), 360)))); //Limelight to 360 TODO
    if(
      std::abs(visionRelative.X().value()) < 1 &&
      std::abs(visionRelative.Y().value()) < 1 &&
      std::abs(-fmod(360 - visionRelative.Rotation().Degrees().value(), 360)) < 30
    )     {
      //DebugOutF("Adjusting" + std::to_string(m_Timer.GetFPGATimestamp().value()));
      if(COB_GET_ENTRY(COB_KEY_TV).GetInteger(0) == 1 && COB_GET_ENTRY(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).size() != 0){
        // DebugOutF("Botpose array");
        // if(COB_GET_ENTRY("COB_KEY_BOT_POSE").GetDoubleArray(std::span<double>()).at(1) <= 3 || COB_GET_ENTRY("COB_KEY_BOT_POSE").GetDoubleArray(std::span<double>()).at(1) >= 13){
        //   DebugOutF("Δx check");

          // m_Odometry.AddVisionMeasurement(frc::Pose2d(Robot::GetRobot()->GetVision().GetPoseBlue().Translation(), m_Rotation), m_Timer.GetFPGATimestamp()
          // - units::second_t((COB_GET_ENTRY("/limelight/tl").GetDouble(0))/1000.0) - units::second_t((COB_GET_ENTRY("/limelight/cl").GetDouble(0))/1000.0)
          // );

        //DebugOutF("Inner Adjusted");
        // }
      }
    }
  }

  //m_Odometry.UpdateWithTime(m_Timer.GetFPGATimestamp(), m_Rotation, m_ModulePositions);
  m_Odometry.Update(m_Rotation, m_ModulePositions);

  // DebugOutF("X: " + std::to_string(m_Odometry.GetEstimatedPosition().X().value()));
  // DebugOutF("Y: " + std::to_string(m_Odometry.GetEstimatedPosition().Y().value()));
  // DebugOutF("Deg: " + std::to_string(m_Odometry.GetEstimatedPosition().Rotation().Degrees().value()));


}

//Converts chassis speed object and updates module states
void DriveTrain::BaseDrive(frc::ChassisSpeeds chassisSpeeds){
  m_ChassisSpeeds = chassisSpeeds;
  // DebugOutF("Y speed: " + std::to_string(Rad2Deg(m_ChassisSpeeds.vy.value())));
  // DebugOutF("Omega: " + std::to_string(Rad2Deg(m_ChassisSpeeds.omega.value())));
  auto [fl, fr, bl, br] = m_Kinematics.ToSwerveModuleStates(m_ChassisSpeeds);
  m_ModuleStates = {fl, fr, bl, br};
}

//Sets breakmode
void DriveTrain::BreakMode(bool on){
  m_FrontLeftModule.BreakMode(on);
  m_FrontRightModule.BreakMode(on);
  m_BackLeftModule.BreakMode(on);
  m_BackRightModule.BreakMode(on);
}

// pathplanner::FollowPathWithEvents* DriveTrain::TruePath(){
//   DriveToPosCommand com = DriveToPosCommand();
//   return new pathplanner::FollowPathWithEvents(std::make_unique<DriveToPosCommand>(com), com.m_Trajectory.getMarkers(), m_EventMap);
// }

// pathplanner::FollowPathWithEvents* DriveTrain::TrueAuto(PathPlannerTrajectory traj){
//   TrajectoryCommand* com = new TrajectoryCommand(traj);
//   return new pathplanner::FollowPathWithEvents(std::make_unique<TrajectoryCommand>(*com), com->m_Trajectory.getMarkers(), m_EventMap);
// }

// void DriveTrain::AutoBalanceFunction(){
//   if(!GetIsBalancing()){
//       frc2::CommandScheduler::GetInstance().Schedule(&m_BalanceCommand);
//       DebugOutF("Start");
//       SetIsBalancing(true);
//     } else if (GetIsBalancing()) {
//       frc2::CommandScheduler::GetInstance().Cancel(&m_BalanceCommand);
//       DebugOutF("End");
//       SetIsBalancing(false);
//     }
// }

// frc2::FunctionalCommand DriveTrain::AutoBalanceCommand(){
//     return frc2::FunctionalCommand(
//       [&]{},
//       [&]{},
//       [&](bool e){
//         AutoBalanceFunction();
//       },
//       [&] {return true;}
//     );
// }

// TrajectoryCommand DriveTrain::DriveToPos(frc::Pose2d target){
//   frc::Pose2d start = m_Odometry.GetEstimatedPosition();
//   DebugOutF("Start: (" + std::to_string(start.Translation().X().value()) + ", " + std::to_string(start.Translation().Y().value()) + ")");
//   DebugOutF("End: (" + std::to_string(target.Translation().X().value()) + ", " + std::to_string(target.Translation().Y().value()) + ")");
//   PathPlannerTrajectory traj = PathPlanner::generatePath(
//     units::meters_per_second_t(0.5),
// 		units::meters_per_second_squared_t(0.25),
    
// 		false, 
//       //  PathPoint(
//       //   m_Odometry.GetEstimatedPosition().Translation(), 
//       //   target.RelativeTo(frc::Pose2d(m_Odometry.GetEstimatedPosition().Translation(), frc::Rotation2d(0_rad))).Rotation(),
//       //   m_Odometry.GetEstimatedPosition().Rotation()
//   //       frc::Translation2d(),
//   //       frc::Rotation2d(),
//   //       frc::Rotation2d()
//       // ), PathPoint(
//       //   target.Translation(), 
//       //   -target.RelativeTo(frc::Pose2d(m_Odometry.GetEstimatedPosition().Translation(), frc::Rotation2d(0_rad))).Rotation(),
//       //   target.Rotation()
//   //       frc::Translation2d(),
//   //       frc::Rotation2d(),
//   //       frc::Rotation2d()
//   // )
//     PathPoint(start.Translation(), frc::Rotation2d(0_deg), frc::Rotation2d(0_deg)), // position, heading(direction of travel), holonomic rotation
//     PathPoint(target.Translation(), frc::Rotation2d(45_deg), frc::Rotation2d(0_deg)
//   ));
  
//   return TrajectoryCommand(traj);
// }

// void DriveTrain::TrajectoryDrive(std::array<frc::SwerveModuleState, 4> states){
//   m_ModuleStates = states;
// } 

// void DriveTrain::TrajectoryFollow(frc::Trajectory trajectory, std::function<frc::Rotation2d()> Rotation){
//     frc2::CommandScheduler::GetInstance().Schedule(new frc2::SwerveControllerCommand<4>{
//       trajectory, 
//       m_Odometry.GetEstimatedPosition(),
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
//     [this]() { return m_Odometry.GetEstimatedPosition(); }, 
//     m_Kinematics, 
//     m_xController, 
//     m_yController, 
//     m_ThetaController, 
//     //[&](){ return trajectory.States()},
//     [this](auto moduleStates) { TrajectoryDrive(moduleStates); }, 
//     {&Robot::s_Instance->GetDriveTrain()}
//   });
// };