// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

constexpr int kDriverControllerPort = 0; //uhhhh idk it just wont build without this line

#define ARM_SUBSYSTEM

#ifdef ARM_SUBSYSTEM

//motor and sensor IDs
#define EXTRACTION_MOTOR 39
#define PIVOT_MOTOR 30
#define LEFT_BRAKE 1
#define RIGHT_BRAKE 0
#define STRINGPOT_ANALOG_INPUT_ID 4

//Math constants
#define PIVOT_GEAR_RATIO 160
#define PIVOT_TICKS_PER_ARM_DEGREE PIVOT_GEAR_RATIO*2048/360
#define SQUEEZE_AMP_THRESHOLD -1 //UPDATE
#define STRING_POT_INCHES_PER_TICK 0.01499326
#define STRING_POT_MINIMUM 138
#define STRING_POT_MAXIMUM 1622

//PID constants
#define PIVOT_ERROR 10
#define PIVOT_KP 0.01 //0.41928
#define PIVOT_KI 0 //DO NOT TOUCH AT ALL (.25 and .01 have broken bot)
#define PIVOT_KD 0.2

#define EXTRACTION_ERROR 0 //check
#define EXTRACTION_KP 0 //check
#define EXTRACTION_KI 0 //check 
#define EXTRACTION_KD 0 //check
//setpoints
#define FRONT_LOW_ANGLE GROUND_PICKUP_ANGLE
#define FRONT_MIDDLE_CONE_ANGLE 24.34
#define FRONT_MIDDLE_CUBE_ANGLE 13.30
#define FRONT_HIGH_CONE_ANGLE 0 //UNUSED
#define FRONT_HIGH_CUBE_ANGLE 0 //UNUSED

#define FRONT_LOW_RADIUS GROUND_PICKUP_RADIUS
#define FRONT_MIDDLE_CONE_RADIUS 53.38
#define FRONT_MIDDLE_CUBE_RADIUS 49.98
#define FRONT_HIGH_CONE_RADIUS 0 //UNUSED
#define FRONT_HIGH_CUBE_RADIUS 0 //UNUSED

#define BACK_LOW_ANGLE 0
#define BACK_MIDDLE_CONE_ANGLE 0 //UNUSED
#define BACK_MIDDLE_CUBE_ANGLE 0 //UNUSED
#define BACK_HIGH_CONE_ANGLE 141.90
#define BACK_HIGH_CUBE_ANGLE 132.70

#define BACK_LOW_RADIUS 0
#define BACK_MIDDLE_CONE_RADIUS 0 //UNUSED
#define BACK_MIDDLE_CUBE_RADIUS 0 //UNUSED
#define BACK_HIGH_CONE_RADIUS 55.10
#define BACK_HIGH_CUBE_RADIUS 63.95

#define TRANSIT_ANGLE 0 //should be zero
#define TRANSIT_RADIUS 42.5

#define FRONT_LOADING_ANGLE 45.62
#define FRONT_LOADING_RADIUS 42.5
#define BACK_LOADING_ANGLE 134.38
#define BACK_LOADING_RADIUS 42.5

#define GROUND_PICKUP_ANGLE -9.57
#define GROUND_PICKUP_RADIUS 42.09

//keywords
#define CONE 0
#define CUBE 1

//button IDs
#define GRID_TL 1 //check
#define GRID_TC 1 //check
#define GRID_TR 1 //check
#define GRID_ML 1 //check
#define GRID_MC 1 //check
#define GRID_MR 1 //check
#define GRID_BL 1 //check
#define GRID_BC 1 //check
#define GRID_BR 1 //check

#define LEFT_GRID 1 //check
#define CENTER_GRID 1 //check
#define RIGHT_GRID 1 //check

#define CONE_MODE 1 //check
#define CUBE_MODE 1 //check

#define FRONT_MODE 1 //check
#define BACK_MODE 1 //check

#define TRANSIT_MODE 1 //check
#define GROUND_PICKUP_MODE 1 //check
#define LOADING_MODE 1 //check

#define PIVOT_CONTROL 1 //check
#define EXTRACTION_CONTROL 1 //check

#define ARM_OVERRIDE 1 //check

#endif