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



using ctre::phoenix::motorcontrol::can::TalonFX;

class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();
  void BaseDrive(frc::ChassisSpeeds chassisSpeeds);
  void DriveInit();
  void BreakMode(bool on);

  void Periodic() override;
    
  // void UseVelocityPID();
  // void UseMagicPID();
  // void UsePostionPID();
  // void SetPID(double E, double P, double I, double D, double F);

  // void DriveToPosition(double x);

  frc::Translation2d m_FrontLeftLocation;
  frc::Translation2d m_FrontRightLocation;
  frc::Translation2d m_BackLeftLocation;
  frc::Translation2d m_BackRightLocation;
  frc::SwerveDriveKinematics<4> m_Kinematics;
  //frc::SwerveDriveOdometry<4> m_Odometry;    //IDK where this was in the code
  frc::Rotation2d m_Rotation;             
  frc::ChassisSpeeds m_ChassisSpeeds;

  SwerveModule m_FrontLeftModule;
  SwerveModule m_FrontRightModule;
  SwerveModule m_BackLeftModule;
  SwerveModule m_BackRightModule;

  const double kMAX_VOLTAGE = 12.0; //FIX

  //how fast the robot should be able to drive
  const units::meters_per_second_t kMAX_VELOCITY_METERS_PER_SECOND = units::meters_per_second_t(6380.0 / 60.0 * DRIVE_REDUCTION * WHEEL_DIAMETER * M_PI);
  
  //theoretical maximum angular velocity - can be replaced with measure amount
  const units::radians_per_second_t kMAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = units::radians_per_second_t(6380.0 / 60.0 * DRIVE_REDUCTION * WHEEL_DIAMETER * M_PI / std::sqrt(Pow((DRIVETRAIN_TRACKWIDTH_METERS / 2), 2) + Pow((DRIVETRAIN_WHEELBASE_METERS / 2), 2)));
};