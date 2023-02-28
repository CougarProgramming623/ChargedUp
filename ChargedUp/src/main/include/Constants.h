// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

constexpr int kDriverControllerPort = 0; //uhhhh idk it just wont build without this line

#define ARM

#ifdef ARM

//motor and sensor IDs
#define EXTRACTION_MOTOR 53
#define PIVOT_MOTOR 32 
#define LEFT_BRAKE 1
#define RIGHT_BRAKE 1
#define STRINGPOT_ANALOG_INPUT_ID 5
#define LEFT_BRAKE 0
#define ROBOT_SABER
//#define ROBOT_WBOT

#define LEFT_GRID_BUTTON -1 // 51
#define MIDDLE_GRID_BUTTON -1 // 52
#define RIGHT_GRID_BUTTON -1 // 53
#define PLACING_BUTTON_A1 -1 //
#define PLACING_BUTTON_A2 -1 // 
#define PLACING_BUTTON_A3 -1 // 
#define PLACING_BUTTON_B1 -1 // 
#define PLACING_BUTTON_B2 -1 // 
#define PLACING_BUTTON_B3 -1 // 
#define PLACING_BUTTON_C1 -1 // 
#define PLACING_BUTTON_C2 -1 // 
#define PLACING_BUTTON_C3 -1 //
#define DRIVE_FRONT_LEFT -1  // 31
#define DRIVE_FRONT_RIGHT 32
#define DRIVE_BACK_LEFT 34
#define DRIVE_BACK_RIGHT 33
#define FLYWHEEL_FRONT 42
#define FLYWHEEL_BACK 41
#define FEEDER 3
#define FEEDER_BUTTON 2
#define FLYWHEEL_BUTTON_BY_DIAL 4
#define FLYWHEEL_BUTTON_BY_DISTANCE 5
#define FLYWHEEL_DIAL -1
#define SHOOTTIME 14
#define TESTBUTTON -1
#define READYSHOOT 3
#define CLIMBPULLUP 31
#define CLIMBPIVOT 3
#endif

#ifdef ROBOT_GEORGE
#define DRIVE_FRONT_LEFT 53
#define DRIVE_FRONT_RIGHT 54
#define DRIVE_BACK_LEFT 51
#define DRIVE_BACK_RIGHT 52
#define FLYWHEEL_FRONT 39
#define FLYWHEEL_BACK 38
#define FEEDER 22
#define FEED_BUTTON 13
#define FLYWHEEL_BUTTON_BY_SPEED 16
#define FLYWHEEL_BUTTON_BY_DISTANCE 14
#define CLIMBPULLUP 30
#define CLIMBPIVOT 15
#define PHOTOSENSOR 3
#define SHOOT_BUTTON 15
#define LIMELIGHT_TOGGLE 1
#define INTAKE_BUTTON_UP_DOWN 6
#define INTAKE_BUTTON_INGEST 7
#define INTAKE_BUTTON_EJECT 8
#define INTAKE_UP_DOWN 11
#define INTAKE_INGEST_EJECT 2
#endif

}  // namespace OperatorConstants
#ifdef ROBOT_SABER

//Math constants
#define PIVOT_GEAR_RATIO 6.75 //6.75 is drive gear ratio//320/1
#define PIVOT_TICKS_PER_ARM_DEGREE 6.75*2048/360
#define EXTRACTION_GEAR_RATIO 1 //NOT CORRECT- UPDATE VARIABLES
#define EXTRACTION_BAR_CIRCUMFERENCE 1 //NOT CORRECT- NEEDS TO BE UPDATED || MUST BE IN INCHES
#define SQUEEZE_AMP_THRESHOLD -1
#define SLIDER_INCHES_PER_TICK 0.00348432055
#define STRING_POT_INCHES_PER_TICK -1 //UPDATE

//keywords
#define CONE 0
#define CUBE 1

//button IDs
#define TELE_NUKE 9
#define UP_DOWN_JOYSTICK 2
#define RELEASE_BUTTON 12
#define FEED_BUTTON 13

#endif