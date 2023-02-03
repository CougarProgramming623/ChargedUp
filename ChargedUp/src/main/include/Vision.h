#pragma once
<<<<<<< HEAD
#include <frc/geometry/Pose2d.h>
=======
#include <frc/apriltag/AprilTagFieldLayout.h>



>>>>>>> 7554f0c19ac13e76d90b0db67e74f798d861674b
class Vision {

    public:
        static void PrintValues();
        static void PushDistance();
    private:
<<<<<<< HEAD
        Pose2d orgin;
        double targetOffsetAngleVertical;
        double totalAngleToTarget;
        double totalRadiansToTarget;
        double totalDistanceInCM;
 
=======
        frc::AprilTagFieldLayout::AprilTagFieldLayout m_FieldLayout;
>>>>>>> 7554f0c19ac13e76d90b0db67e74f798d861674b

};