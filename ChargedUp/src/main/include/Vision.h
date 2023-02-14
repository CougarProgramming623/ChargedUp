#pragma once
#include <frc/geometry/Pose2d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>



class Vision {

    public:
        Vision();
        void PrintValues();
        void PushDistance();
        void VisionInit();
        double GetDistance();
    private:
        //Pose2d orgin;
        double m_TargetOffsetAngleVertical;
        double m_TotalAngleToTarget;
        double m_TotalRadiansToTarget;
        double m_TotalDistanceInCM;

        //represents the april tags ont he field for the push distance method that way an 
        //if statement that checks index ID returns true if short and fale if tall
        const bool kAprilTagID[9] = {false, true, true, true, false, false, true, true, true};
 
        //frc::AprilTagFieldLayout::AprilTagFieldLayout m_FieldLayout;

};