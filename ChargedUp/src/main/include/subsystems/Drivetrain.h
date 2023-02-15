#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/geometry/Rotation2d.h>


//copied includes
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/HolonomicDriveController.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/angular_acceleration.h>
#include <units/math.h>
#include <array>
#include <fstream>
#include "./Util.h"
#include "Constants.h"
#include "SwerveModule.h"
#include <frc2/command/SubsystemBase.h>
#include "commands/DriveWithJoystick.h"
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <pathplanner/lib/PathPlanner.h>

using ctre::phoenix::motorcontrol::can::TalonFX;

class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();
  void BaseDrive(frc::ChassisSpeeds chassisSpeeds);
  void DriveInit();
  void BreakMode(bool on);
  // void TrajectoryFollow(frc::Trajectory trajectory);
  // void TrajectoryDrive(std::array<frc::SwerveModuleState, 4> states);
  // void PathPlannerFollow(pathplanner::PathPlannerTrajectory trajectory);

  void Periodic() override;

  frc::Translation2d m_FrontLeftLocation;
  frc::Translation2d m_FrontRightLocation;
  frc::Translation2d m_BackLeftLocation;
  frc::Translation2d m_BackRightLocation;

  inline frc::SwerveDriveKinematics<4> GetKinematics() { return m_Kinematics; }
  inline frc::SwerveDriveOdometry<4> GetOdometry(){ return m_Odometry; }
  inline frc::HolonomicDriveController GetHolonomicController(){ return m_HolonomicController; }

  const inline std::array<frc::SwerveModulePosition, 4> GetModulePositions(){ return m_ModulePositions; }

//how fast the robot should be able to drive
  const units::meters_per_second_t kMAX_VELOCITY_METERS_PER_SECOND = units::meters_per_second_t(6380.0 / 60.0 * DRIVE_REDUCTION * WHEEL_DIAMETER * M_PI);

  std::array<frc::SwerveModulePosition, 4> m_ModulePositions = {m_FrontLeftModule.GetPosition(), m_FrontRightModule.GetPosition(), m_BackLeftModule.GetPosition(), m_BackRightModule.GetPosition()};

  const double kMAX_VOLTAGE = 12.0; //FIX
  
  SwerveModule m_FrontLeftModule;
  SwerveModule m_FrontRightModule;
  SwerveModule m_BackLeftModule;
  SwerveModule m_BackRightModule;

  //theoretical maximum angular velocity - can be replaced with measure amount
  const units::radians_per_second_t kMAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = units::radians_per_second_t(6380.0 / 60.0 * DRIVE_REDUCTION * WHEEL_DIAMETER * M_PI / std::sqrt(Pow((DRIVETRAIN_TRACKWIDTH_METERS / 2), 2) + Pow((DRIVETRAIN_WHEELBASE_METERS / 2), 2)));

  private:

  frc::SwerveDriveKinematics<4> m_Kinematics;
  frc::SwerveDriveOdometry<4> m_Odometry;
  
  frc::Rotation2d m_Rotation;             
  frc::ChassisSpeeds m_ChassisSpeeds;

  std::array<frc::SwerveModuleState, 4> m_ModuleStates;
  
  frc2::PIDController m_xController;
  frc2::PIDController m_yController;
  frc::ProfiledPIDController <units::radians> m_ThetaController;
  frc::HolonomicDriveController m_HolonomicController;
};