#pragma once 

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h" //idk

#define LIMELIGHT_HEIGHT 0  // 78.74 //cm
#define TARGET_HEIGHT    0    // cm
#define LIMELIGHT_ANGLE  0

class COB {
 public:
  inline nt::NetworkTableInstance& GetTable() { return m_Table; }

 private:
  nt::NetworkTableInstance m_Table = nt::NetworkTableInstance::GetDefault();
};