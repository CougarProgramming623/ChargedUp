#pragma once
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>


using namespace frc;

class Vision {

    public:
        Vision();
        void PrintValues();
        void VisionInit();
        void CalcPose();

        Pose2d GetPose();

    private:
        double m_TargetOffsetAngleVertical;
        double m_TotalAngleToTarget;
        double m_TotalRadiansToTarget;
        double m_TotalDistanceInCM;
        double m_BotPose[6];

        Pose2d m_AbsolutePose;

        



};