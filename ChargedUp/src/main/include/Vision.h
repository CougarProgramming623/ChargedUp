#pragma once
#include <frc/geometry/Pose2d.h>
class Vision {

    public:
        static void PrintValues();
        static void PushDistance();
    private:
        Pose2d orgin;
        double targetOffsetAngleVertical;
        double totalAngleToTarget;
        double totalRadiansToTarget;
        double totalDistanceInCM;
 

};