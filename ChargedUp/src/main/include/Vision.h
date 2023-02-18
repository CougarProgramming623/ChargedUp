#pragma once
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>


using namespace frc;

class Vision {

    public:
        Vision();
        void PrintValues();
        void PushDistance();
        void VisionInit();
        double GetDistance();
        Pose2d GetPose();

    private:
        double m_TargetOffsetAngleVertical;
        double m_TotalAngleToTarget;
        double m_TotalRadiansToTarget;
        double m_TotalDistanceInCM;

        Pose2d m_AbsolutePose;

        Pose2d AprilTagRelativePose();
        Pose2d CreateAbsolutePose();

        //represents the april tags ont he field for the push distance method that way an 
        //if statement that checks index ID returns true if short and fale if tall
        const bool kAprilTagID[9] = {false, true, true, true, false, false, true, true, true};

        //the transformations to convert apriltag relative pose to absolute pose
        const Transform2d kTransform[9] = {};
 
        //frc::AprilTagFieldLayout::AprilTagFieldLayout m_FieldLayout;

};