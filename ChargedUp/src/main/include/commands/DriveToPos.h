#pragma once
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>


class DriveToPos : public frc2::CommandHelper<frc2::CommandBase, DriveToPos> {
public:
    explicit DriveToPos(frc::Pose2d targetPose, frc::Rotation2d targetAngle);

    void Initialize() override;
    void Execute() override;

    frc::Pose2d m_Current;
    frc::Pose2d m_TargetPose;
    frc::Rotation2d m_TargetAngle; 
};