#include "Vision.h"
#include "Robot.h"
#include "Util.h"

void Vision::PrintValues() {
//std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
double tx = Robot::GetRobot()->GetCOB().GetTable().GetEntry("/limelight/tx").GetDouble(0.0);
double ty = Robot::GetRobot()->GetCOB().GetTable().GetEntry("/limelight/ty").GetDouble(0.0);
double tv = Robot::GetRobot()->GetCOB().GetTable().GetEntry("/limelight/tv").GetDouble(0.0);

DebugOutF("tx: " + std::to_string(tx));
DebugOutF("ty: " + std::to_string(ty));
DebugOutF("tv: " + std::to_string(tv));
}