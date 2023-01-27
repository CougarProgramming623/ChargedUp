// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

constexpr int kDriverControllerPort = 0; //uhhhh idk it just wont build without this line

#define ARM

#ifdef ARM

#define EXTRACTION_MOTOR -1
#define PIVOT_MOTOR -1
#define LEFT_BRAKE -1
#define RIGHT_BRAKE -1

#define PIVOT_GEAR_RATIO 320/1
#define PIVOT_TICKS_PER_ARM_DEGREE 1820.444

#define EXTRACTION_GEAR_RATIO -1 //NOT CORRECT- UPDATE VARIABLES
#define EXTRACTION_BAR_CIRCUMFERENCE -1 //NOT CORRECT- NEEDS TO BE UPDATED || MUST BE IN INCHES

#endif