#pragma once
#include <frc/geometry/Pose2d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>



class Vision {

    public:
        static void PrintValues();
        static void PushDistance();
    private:
        //Pose2d orgin;
        double targetOffsetAngleVertical;
        double totalAngleToTarget;
        double totalRadiansToTarget;
        double totalDistanceInCM;
 
        //frc::AprilTagFieldLayout::AprilTagFieldLayout m_FieldLayout;

};