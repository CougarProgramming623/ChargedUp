#include "Vision.h"
#include "Robot.h"
#include "Util.h"

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
    targetOffsetAngleVertical = GetRobot()->GetCOB().GetTable().GetEntry("/limelight/ty").GetDouble(0.0);
    totalAngleToTarget = LIMELIGHT_ANGLE + targetOffsetAngleVertical;
    totalRadiansToTarget = Deg2Rad(totalAngleToTarget);

    if(true)/{//Short Target
    totalDistanceInCM =(TARGET_HEIGHT_SHORT - LIMELIGHT_HEIGHT)/cmath::tan(totalRadiansToTarget)
    GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_DISTANCE).SetDouble(totalDistanceInCM);
    }
    else if(false){//Tall Target
    totalDistanceInCM =(TARGET_HEIGHT_TALL - LIMELIGHT_HEIGHT)/cmath::tan(totalRadiansToTarget)
    GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_DISTANCE).SetDouble(totalDistanceInCM);
    }

 }
 /*

 #define LIMELIGHT_HEIGHT 60.16625  // 78.74 //cm
    #define TARGET_HEIGHT_TALL  69.16    // Loading Zone /cm
    #define TARGET_HEIGHT_SHORT 46.16   // Grid //cm
    #define LIMELIGHT_ANGLE  15.30 */