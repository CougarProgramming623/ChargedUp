#include "Vision.h"
#include "Robot.h"
#include "Util.h"
#include <math.h>

Vision::Vision(){}
void Vision::VisionInit(){}

void Vision::PrintValues() {
// std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
double tx = Robot::GetRobot()->GetCOB().GetTable().GetEntry("/limelight/tx").GetDouble(-1);
double ty = Robot::GetRobot()->GetCOB().GetTable().GetEntry("/limelight/ty").GetDouble(-1);
double tv = Robot::GetRobot()->GetCOB().GetTable().GetEntry("/limelight/tv").GetDouble(-1);

DebugOutF("tx: " + std::to_string(tx));
DebugOutF("ty: " + std::to_string(ty));
DebugOutF("tv: " + std::to_string(tv));
}

/*Equation d = (h2 - h1) / tan(a1 + a2)
    h1 = height of lime light
    h2 = height of center of traget 
    a1 = angle of limelight
    a2 = angle of target to limelight ty
*/
 void Vision::PushDistance() {
    m_TargetOffsetAngleVertical = Robot::GetRobot()->GetCOB().GetTable().GetEntry("/limelight/ty").GetDouble(0.0);
    m_TotalAngleToTarget = LIMELIGHT_ANGLE + m_TargetOffsetAngleVertical;
    m_TotalRadiansToTarget = Deg2Rad(m_TotalAngleToTarget);

    if(true){//kAprilTagID[Robot::GetRobot()->GetCOB().GetTable().GetEntry("/limelight/tid").GetBoolean(false)]){//Short Target
    //DebugOutF("Ty" + std::to_string(m_TargetOffsetAngleVertical));
    m_TotalDistanceInCM =(TARGET_HEIGHT_SHORT - LIMELIGHT_HEIGHT)/tan(std::abs(m_TotalRadiansToTarget));
    Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_DISTANCE).SetDouble(m_TotalDistanceInCM);
    }
   //  else if(!kAprilTagID[Robot::GetRobot()->GetCOB().GetTable().GetEntry("/limelight/tid").GetBoolean(true)]){//Tall Target
    
   //  m_TotalDistanceInCM =(TARGET_HEIGHT_TALL - LIMELIGHT_HEIGHT)/tan(std::abs(m_TotalRadiansToTarget));
   //  Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_DISTANCE).SetDouble(m_TotalDistanceInCM);
   //  }

 }
