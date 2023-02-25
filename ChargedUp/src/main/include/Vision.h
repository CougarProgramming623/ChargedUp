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
        
        Pose2d GetPoseBlue();
        Pose2d GetPoseRed();
        Pose2d GetfieldPose();


    private:
        double m_TargetOffsetAngleVertical;
        double m_TotalAngleToTarget;
        double m_TotalRadiansToTarget;
        double m_TotalDistanceInCM;
        double m_BotPose[6];

        int m_timer = 0;

        Pose2d m_AbsolutePose;
        const Pose2d kBlueOrigin = Pose2d(units::meter_t(-813.816), units::meter_t(-400.812), units::radian_t(0)); 
        const Pose2d kRedOrigin = Pose2d(units::meter_t(0), units::meter_t(0), units::radian_t(0)); //FIX

        



};