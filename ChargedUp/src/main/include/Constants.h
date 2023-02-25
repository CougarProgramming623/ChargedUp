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