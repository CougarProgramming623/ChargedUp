#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>



class Vision {

    public:
          static void PrintValues();

    private:
        frc::AprilTagFieldLayout::AprilTagFieldLayout m_FieldLayout;

};