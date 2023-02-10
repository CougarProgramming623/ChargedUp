#pragma once
#include <frc/geometry/Pose2d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>



class Vision {

    public:
        Vision();
        void PrintValues();
        void PushDistance();
        void VisionInit();
    private:
        //Pose2d orgin;
        double m_TargetOffsetAngleVertical;
        double m_TotalAngleToTarget;
        double m_TotalRadiansToTarget;
        double m_TotalDistanceInCM;
 
        //frc::AprilTagFieldLayout::AprilTagFieldLayout m_FieldLayout;

};