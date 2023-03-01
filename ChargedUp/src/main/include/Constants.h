// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <math.h>

//#define ROBOT_WHEELBASE
#define O12

#define DRIVE_REDUCTION ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0))
#define STEER_REDUCTION ((14.0 /50.0) * (10.0 / 60.0))

#define STEER_ENCODER_POSITION_CONSTANT (2.0 * M_PI / 2048 * STEER_REDUCTION)
#define STEER_ENCODER_VELOCITY_CONSTANT (STEER_ENCODER_POSITION_CONSTANT * 10.0)

#define ENCODER_RESET_MAX_ANGULAR_VELOCITY (Deg2Rad(0.5))
#define ENCODER_RESET_ITERATIONS 500

#define WHEEL_DIAMETER 0.10033

#ifdef ROBOT_WHEELBASE

    #define DRIVETRAIN_TRACKWIDTH_METERS 0.5644

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    #define DRIVETRAIN_WHEELBASE_METERS 0.666

    #define FRONT_LEFT_MODULE_DRIVE_MOTOR 32
    #define FRONT_LEFT_MODULE_STEER_MOTOR 35
    //#define FRONT_LEFT_MODULE_STEER_OFFSET -Deg2Rad(0.0)
    #define FRONT_LEFT_MODULE_ENCODER_PORT 3

    #define FRONT_RIGHT_MODULE_DRIVE_MOTOR 37
    #define FRONT_RIGHT_MODULE_STEER_MOTOR 33
    //#define FRONT_RIGHT_MODULE_STEER_OFFSET -Deg2Rad(0.0)
    #define FRONT_RIGHT_MODULE_ENCODER_PORT 1

    #define BACK_LEFT_MODULE_DRIVE_MOTOR 41
    #define BACK_LEFT_MODULE_STEER_MOTOR 36
    //#define BACK_LEFT_MODULE_STEER_OFFSET -Deg2Rad(0.0)
    #define BACK_LEFT_MODULE_ENCODER_PORT 2

    #define BACK_RIGHT_MODULE_DRIVE_MOTOR 42
    #define BACK_RIGHT_MODULE_STEER_MOTOR 34
    //#define BACK_RIGHT_MODULE_STEER_OFFSET -Deg2Rad(0.0)
    #define BACK_RIGHT_MODULE_ENCODER_PORT 0


    #define LIMELIGHT_HEIGHT    38.57625   //cm
    //#define LIMELIGHT_HEIGHT 44.846875 //cm on C
    #define TARGET_HEIGHT_TALL  69    // Loading Zone //cm
    //#define TARGET_HEIGHT_SHORT 31.4   // Grid //cm
    #define TARGET_HEIGHT_SHORT 31.59125 //cm on the cone roughly 12.5 in
    #define LIMELIGHT_ANGLE  2.25     // degrees
#endif

#ifdef O12

    #define DRIVETRAIN_TRACKWIDTH_METERS 0.61595

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    #define DRIVETRAIN_WHEELBASE_METERS 0.61595

    #define FRONT_LEFT_MODULE_DRIVE_MOTOR 54
    #define FRONT_LEFT_MODULE_STEER_MOTOR 58
    //#define FRONT_LEFT_MODULE_STEER_OFFSET -Deg2Rad(0.0)
    #define FRONT_LEFT_MODULE_ENCODER_PORT 2

    #define FRONT_RIGHT_MODULE_DRIVE_MOTOR 38
    #define FRONT_RIGHT_MODULE_STEER_MOTOR 61
    //#define FRONT_RIGHT_MODULE_STEER_OFFSET -Deg2Rad(0.0)
    #define FRONT_RIGHT_MODULE_ENCODER_PORT 1

    #define BACK_LEFT_MODULE_DRIVE_MOTOR 52
    #define BACK_LEFT_MODULE_STEER_MOTOR 59
    //#define BACK_LEFT_MODULE_STEER_OFFSET -Deg2Rad(0.0)
    #define BACK_LEFT_MODULE_ENCODER_PORT 3

    #define BACK_RIGHT_MODULE_DRIVE_MOTOR 51
    #define BACK_RIGHT_MODULE_STEER_MOTOR 60
    //#define BACK_RIGHT_MODULE_STEER_OFFSET -Deg2Rad(0.0)
    #define BACK_RIGHT_MODULE_ENCODER_PORT 0


    #define LIMELIGHT_HEIGHT    -1   //cm
    //#define LIMELIGHT_HEIGHT 44.846875 //cm on C
    //#define TARGET_HEIGHT_SHORT 31.4   // Grid //cm
    #define LIMELIGHT_ANGLE  -1     // degrees


#endif

#define TARGET_HEIGHT_TALL  69    // Loading Zone //cm
#define TARGET_HEIGHT_SHORT 31.59125 //cm on the cone roughly 12.5 in

#define DRIVE_ENCODER_POSITION_CONSTANT (M_PI * WHEEL_DIAMETER * DRIVE_REDUCTION / 2048)
#define DRIVE_ENCODER_VELOCITY_CONSTANT (DRIVE_ENCODER_POSITION_CONSTANT * 10)

#define ENCODER_VOLTAGE_TO_DEGREE (360/4.809)

#define COB_GET_ENTRY Robot::GetRobot()->GetCOB().GetTable().GetEntry
    
#define COB_KEY_IS_RED   "/FMSInfo/IsRedAlliance"

#define COB_KEY_DISTANCE "/COB/distance"
#define COB_KEY_BOT_POSE "/limelight/botpose"
#define COB_KEY_TV "/limelight/tv"