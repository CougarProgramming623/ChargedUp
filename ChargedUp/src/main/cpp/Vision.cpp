#include "Vision.h"
#include "Robot.h"
#include "Util.h"

#include <math.h>
#include <LimelightHelpers.h>
#include <frc/geometry/Rotation2d.h>

using namespace frc;

Vision::Vision(){}
void Vision::VisionInit(){
  
}

void Vision::PrintValues() {
  double x = (double)m_AbsolutePose.X();
  double y = (double)m_AbsolutePose.Y();

  if(x != 0 && y != 0){
    DebugOutF("tx: " + std::to_string(x));
    DebugOutF("ty: " + std::to_string(y));
  }
}


void Vision::CalcPose(){
  // DebugOutF("calc Pose");

  if(COB_GET_ENTRY(COB_KEY_TV).GetInteger(0) == 1 && COB_GET_ENTRY(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).size() != 0){
    
    //print size
    //DebugOutF("size: " + std::to_string((int)Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).size()));
    // DebugOutF("Tv: "   + std::to_string((int)Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_TV).GetInteger(0)));


    // //print x, y, theta
    // DebugOutF("\nx: " + std::to_string((int)Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).at(1)));
    // DebugOutF("y: " + std::to_string((int)Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).at(0)));
    // DebugOutF("θ: " + std::to_string((int)Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).at(5)));

    //Store current posisition in a pose2d
    m_AbsolutePose = GetFieldPose();
    //Blue 
    DebugOutF("Blue");
    m_AbsolutePose = m_AbsolutePose.RelativeTo(kBlueOrigin);
  }
}

Pose2d Vision::GetPoseBlue(){
  m_AbsolutePose = GetFieldPose();
  m_AbsolutePose = m_AbsolutePose.RelativeTo(kBlueOrigin);

  return m_AbsolutePose;
}

Pose2d Vision::GetPoseRed(){
  m_AbsolutePose = GetFieldPose();
  m_AbsolutePose = m_AbsolutePose.RelativeTo(kRedOrigin);

  return m_AbsolutePose;
}

Pose2d Vision::GetFieldPose(){ 
  if(COB_GET_ENTRY(COB_KEY_TV).GetInteger(0) == 1 && COB_GET_ENTRY(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).size() != 0){
    m_AbsolutePose = Pose2d(  units::meter_t(COB_GET_ENTRY(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).at(0)),
                              units::meter_t(COB_GET_ENTRY(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).at(1)),
                              Rotation2d(units::radian_t(Deg2Rad(COB_GET_ENTRY(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).at(5))))
                            );
  }
  return m_AbsolutePose; 
}