// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

constexpr int kDriverControllerPort = 0;

#define TELE_NUKE 9
#define UP_DOWN_JOYSTICK 2
#define RELEASE_BUTTON 12
#define FEED_BUTTON 13

#define ROBOT_SABER
//#define ROBOT_WBOT

#ifdef ROBOT_SABER
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
