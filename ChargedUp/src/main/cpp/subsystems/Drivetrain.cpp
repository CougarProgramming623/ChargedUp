#include "subsystems/Drivetrain.h"
#include "Robot.h"

#include <frc/trajectory/Trajectory.h>
#include <frc/kinematics/SwerveModuleState.h>

DriveTrain::DriveTrain()
    : m_FrontLeftLocation(units::meter_t (DRIVETRAIN_TRACKWIDTH_METERS / 2.0), units::meter_t (-DRIVETRAIN_WHEELBASE_METERS / 2.0)),
      m_FrontRightLocation(units::meter_t (DRIVETRAIN_TRACKWIDTH_METERS / 2.0), units::meter_t (DRIVETRAIN_WHEELBASE_METERS / 2.0)),
      m_BackLeftLocation(units::meter_t (-DRIVETRAIN_TRACKWIDTH_METERS / 2.0), units::meter_t (-DRIVETRAIN_WHEELBASE_METERS / 2.0)),
      m_BackRightLocation(units::meter_t (-DRIVETRAIN_TRACKWIDTH_METERS / 2.0), units::meter_t (DRIVETRAIN_WHEELBASE_METERS / 2.0)),
      m_Kinematics(m_FrontLeftLocation, m_FrontRightLocation, m_BackLeftLocation, m_BackRightLocation),
      m_Rotation(),
      m_Odometry(m_Kinematics, m_Rotation, wpi::array<frc::SwerveModulePosition, 4>
        (m_FrontLeftModule.GetPosition(), m_FrontRightModule.GetPosition(), m_BackLeftModule.GetPosition(), m_BackRightModule.GetPosition())),
      m_FrontLeftModule(FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_ENCODER_PORT, -137),
      m_FrontRightModule(FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_ENCODER_PORT, -287),
      m_BackLeftModule(BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_ENCODER_PORT, -140.6),
      m_BackRightModule(BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_ENCODER_PORT, -2),
      m_ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s}
{}

void DriveTrain::Periodic(){
  auto [fl, fr, bl, br] = m_Kinematics.ToSwerveModuleStates(m_ChassisSpeeds);
  frc::SwerveModuleState states[4] = {fl, fr, bl, br};

  if((states[0].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE) == 0 && ((double) states[0].angle.Radians() == 0)){
    m_FrontLeftModule.m_SteerController.motor.Set(ControlMode::PercentOutput, 0);
    m_FrontLeftModule.m_DriveController.motor.Set(ControlMode::PercentOutput, 0);
  } else {
    m_FrontLeftModule.Set(states[0].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE, (double) states[0].angle.Radians());
  }

  if((states[1].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE == 0) && ((double) states[1].angle.Radians() == 0)){
    m_FrontRightModule.m_SteerController.motor.Set(ControlMode::PercentOutput, 0);
    m_FrontRightModule.m_DriveController.motor.Set(ControlMode::PercentOutput, 0);
  } else {
    m_FrontRightModule.Set(states[1].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE, (double) states[1].angle.Radians());
  }

  if((states[2].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE == 0) && ((double) states[2].angle.Radians() == 0)){
    m_BackLeftModule.m_SteerController.motor.Set(ControlMode::PercentOutput, 0);
    m_BackLeftModule.m_DriveController.motor.Set(ControlMode::PercentOutput, 0);
  } else {
    m_BackLeftModule.Set(states[2].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE, (double) states[2].angle.Radians());
  }

  if((states[3].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE == 0) && ((double) states[3].angle.Radians() == 0)){
    m_BackRightModule.m_SteerController.motor.Set(ControlMode::PercentOutput, 0);
    m_BackRightModule.m_DriveController.motor.Set(ControlMode::PercentOutput, 0);
  } else {
    m_BackRightModule.Set(states[3].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE, (double) states[3].angle.Radians());
  }

  m_Odometry.Update(m_Rotation, wpi::array<frc::SwerveModulePosition, 4>
        (m_FrontLeftModule.GetPosition(), m_FrontRightModule.GetPosition(), m_BackLeftModule.GetPosition(), m_BackRightModule.GetPosition()));
}

void DriveTrain::BaseDrive(frc::ChassisSpeeds chassisSpeeds){
  m_ChassisSpeeds = chassisSpeeds;
}
void DriveTrain::DriveInit(){
  m_Rotation = frc::Rotation2d(units::radian_t(Robot::s_Instance->GetNavX().GetAngle()));
  SetDefaultCommand(DriveWithJoystick());
}

void DriveTrain::BreakMode(bool on){
  m_FrontLeftModule.BreakMode(true);
  m_FrontRightModule.BreakMode(true);
  m_BackLeftModule.BreakMode(true);
  m_BackRightModule.BreakMode(true);
}
