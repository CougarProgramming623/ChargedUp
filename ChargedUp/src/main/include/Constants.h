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
#define RIGHT_BRAKE 0
#define STRINGPOT_ANALOG_INPUT_ID 5

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
#define GRID_TL 1
#define GRID_TC 1
#define GRID_TR 1
#define GRID_ML 1
#define GRID_MC 1
#define GRID_MR 1 
#define GRID_BL 1
#define GRID_BC 1
#define GRID_BR 1

#define LEFT_GRID 1
#define CENTER_GRID 1
#define RIGHT_GRID 1

#define CONE_MODE 1
#define CUBE_MODE 1

#define FRONT_MODE 1
#define BACK_MODE 1

#define TRANSIT_MODE 1
#define GROUND_PICKUP_MODE 1
#define LOADING_MODE 1

#define PIVOT_CONTROL 1
#define EXTRACTION_CONTROL 1


#endif