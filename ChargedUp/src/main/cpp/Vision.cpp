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
//double x = m_AbsolutePose.X;
//double y = m_AbsolutePose.Y;

DebugOutF("tx: " + std::to_string((double) m_AbsolutePose.X()));
DebugOutF("ty: " + std::to_string((double) m_AbsolutePose.Y()));
}

Pose2d Vision::GetPose(){ 
  CalcPose();
  return m_AbsolutePose; 
}

void Vision::CalcPose(){

  DebugOutF(std::to_string((int)Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).at(1)));
  DebugOutF(std::to_string((int)Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).at(0)));
  DebugOutF(std::to_string((int)Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).at(5)));

 
  if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
    //Red
   m_AbsolutePose = Pose2d(  units::meter_t(Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).at(0)),
                              units::meter_t(Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).at(1)),
                              Rotation2d(units::radian_t(Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).at(5)))
                            );
  }// else {
  //   //Blue 
  //   m_AbsolutePose = Pose2d(  units::meter_t(LimelightHelpers::getBotpose_wpiBlue().at(0)),
  //                             units::meter_t(LimelightHelpers::getBotpose_wpiBlue().at(1)),
  //                             Rotation2d(units::radian_t(LimelightHelpers::getBotpose_wpiBlue().at(5)))
  //                           );
  // }

}



