#include "Vision.h"
#include "Robot.h"
#include "Util.h"

#include <math.h>
#include <LimelightHelpers.h>
#include <frc/geometry/Rotation2d.h>

using namespace frc;

Vision::Vision(){}
void Vision::VisionInit(){}

void Vision::PrintValues() {
// // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
// double x = m_AbsolutePose.X;
// double y = m_AbsolutePose.y;
// double z = m_AbsolutePose.

// DebugOutF("tx: " + std::to_string(x));
// DebugOutF("ty: " + std::to_string(y));
// DebugOutF("tv: " + std::to_string(v));
}

Pose2d Vision::GetPose(){ 
  CalcPose();
  return m_AbsolutePose; 
}

void Vision::CalcPose(){
 
  if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
    //Red
    m_AbsolutePose = Pose2d(  units::meter_t(LimelightHelpers::getBotpose_wpiRed().at(0)),
                              units::meter_t(LimelightHelpers::getBotpose_wpiRed().at(1)),
                              Rotation2d(units::radian_t(LimelightHelpers::getBotpose_wpiRed().at(5)))
                            );
  } else {
    //Blue 
    m_AbsolutePose = Pose2d(  units::meter_t(LimelightHelpers::getBotpose_wpiBlue().at(0)),
                              units::meter_t(LimelightHelpers::getBotpose_wpiBlue().at(1)),
                              Rotation2d(units::radian_t(LimelightHelpers::getBotpose_wpiBlue().at(5)))
                            );
  }

}



