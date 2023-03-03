// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <math.h>

#define ROBOT_WHEELBASE

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
    #define FRONT_LEFT_MODULE_STEER_OFFSET -137
    #define FRONT_LEFT_MODULE_ENCODER_PORT 3

    #define FRONT_RIGHT_MODULE_DRIVE_MOTOR 37
    #define FRONT_RIGHT_MODULE_STEER_MOTOR 33
    #define FRONT_RIGHT_MODULE_STEER_OFFSET -287
    #define FRONT_RIGHT_MODULE_ENCODER_PORT 1

    #define BACK_LEFT_MODULE_DRIVE_MOTOR 41
    #define BACK_LEFT_MODULE_STEER_MOTOR 36
    #define BACK_LEFT_MODULE_STEER_OFFSET -140.6
    #define BACK_LEFT_MODULE_ENCODER_PORT 2

    #define BACK_RIGHT_MODULE_DRIVE_MOTOR 42
    #define BACK_RIGHT_MODULE_STEER_MOTOR 34
    #define BACK_RIGHT_MODULE_STEER_OFFSET -2
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
    #define FRONT_LEFT_MODULE_STEER_OFFSET -1
    #define FRONT_LEFT_MODULE_ENCODER_PORT 2

    #define FRONT_RIGHT_MODULE_DRIVE_MOTOR 38
    #define FRONT_RIGHT_MODULE_STEER_MOTOR 61
    #define FRONT_RIGHT_MODULE_STEER_OFFSET -1
    #define FRONT_RIGHT_MODULE_ENCODER_PORT 1

    #define BACK_LEFT_MODULE_DRIVE_MOTOR 52
    #define BACK_LEFT_MODULE_STEER_MOTOR 59
    #define BACK_LEFT_MODULE_STEER_OFFSET -1
    #define BACK_LEFT_MODULE_ENCODER_PORT 3

    #define BACK_RIGHT_MODULE_DRIVE_MOTOR 51
    #define BACK_RIGHT_MODULE_STEER_MOTOR 60
    #define BACK_RIGHT_MODULE_STEER_OFFSET -1
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

constexpr int kDriverControllerPort = 0; //uhhhh idk it just wont build without this line

#define ARM_SUBSYSTEM

#ifdef ARM_SUBSYSTEM

//motor and sensor IDs
#define EXTRACTION_MOTOR 39
#define PIVOT_MOTOR 30
#define LEFT_BRAKE 0
#define RIGHT_BRAKE 1
#define SLIP_BRAKE 2
#define STRINGPOT_ANALOG_INPUT_ID 4

//Math constants
#define PIVOT_GEAR_RATIO 160
#define PIVOT_TICKS_PER_ARM_DEGREE (PIVOT_GEAR_RATIO*2048/360)
#define SQUEEZE_AMP_THRESHOLD .5
#define STRING_POT_INCHES_PER_TICK 0.01499326
#define STRING_POT_MINIMUM 138  
#define STRING_POT_MAXIMUM 1622
#define EXTRACTION_MOTOR_HOLD_POWER .075
#define ARM_MINIMUM_LENGTH 42.5
#define ARM_MAXIMUM_LENGTH 65

//PID constants
#define PIVOT_ERROR 10
#define PIVOT_KP 0.005 //0.41928
#define PIVOT_KI 0 //DO NOT TOUCH AT ALL (.25 and .01 have broken bot)
#define PIVOT_KD 0.2

#define EXTRACTION_ERROR 0 //check
#define EXTRACTION_KP 0 //check
#define EXTRACTION_KI 0 //check 
#define EXTRACTION_KD 0 //check
//setpoints
#define FRONT_LOW_ANGLE GROUND_PICKUP_ANGLE
#define FRONT_MIDDLE_CONE_ANGLE -65.66
#define FRONT_MIDDLE_CUBE_ANGLE -76.70
#define FRONT_HIGH_CONE_ANGLE 0 //UNUSED
#define FRONT_HIGH_CUBE_ANGLE 0 //UNUSED

#define FRONT_LOW_RADIUS GROUND_PICKUP_RADIUS
#define FRONT_MIDDLE_CONE_RADIUS 53.38
#define FRONT_MIDDLE_CUBE_RADIUS 49.98
#define FRONT_HIGH_CONE_RADIUS 0 //UNUSED
#define FRONT_HIGH_CUBE_RADIUS 0 //UNUSED

#define BACK_LOW_ANGLE 0 //UNUSED
#define BACK_MIDDLE_CONE_ANGLE 0 //UNUSED
#define BACK_MIDDLE_CUBE_ANGLE 0 //UNUSED
#define BACK_HIGH_CONE_ANGLE 51.90
#define BACK_HIGH_CUBE_ANGLE 42.70

#define BACK_LOW_RADIUS 0 //UNUSED
#define BACK_MIDDLE_CONE_RADIUS 0 //UNUSED
#define BACK_MIDDLE_CUBE_RADIUS 0 //UNUSED
#define BACK_HIGH_CONE_RADIUS 55.10
#define BACK_HIGH_CUBE_RADIUS 63.95

#define TRANSIT_ANGLE 0 //should be zero
#define TRANSIT_RADIUS ARM_MINIMUM_LENGTH

#define FRONT_LOADING_ANGLE -62.14
#define FRONT_LOADING_RADIUS ARM_MAXIMUM_LENGTH 
#define BACK_LOADING_ANGLE 44.38
#define BACK_LOADING_RADIUS ARM_MINIMUM_LENGTH

#define GROUND_PICKUP_ANGLE -99.57
#define GROUND_PICKUP_RADIUS 42.09

//keywords
#define CONE 0
#define CUBE 1

//button IDs
//BUTTONBOARD 0
#define CONE_MODE 16 
#define CUBE_MODE 15 

#define FRONT_MODE 1 //check
#define BACK_MODE 20 

#define PIVOT_CONTROL 1 //check
#define EXTRACTION_CONTROL 1 //check
#define MANUAL_ARM_BRAKE 22
#define MANUAL_SLIP_BRAKE 7 


#define ARM_OVERRIDE 1 

//BUTTONBOARD 2
#define GRID_TL 2
#define GRID_TC 7 
#define GRID_TR 12
#define GRID_ML 3 
#define GRID_MC 8 
#define GRID_MR 10
#define GRID_BL 4
#define GRID_BC 9
#define GRID_BR 14

#define TRANSIT_MODE 13 
#define GROUND_PICKUP_MODE 5
#define LOADING_MODE 15 

#define LEFT_GRID 1 
#define CENTER_GRID 6 
#define RIGHT_GRID 11

#endif