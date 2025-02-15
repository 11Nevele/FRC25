#pragma once

#include <frc/apriltag/AprilTag.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose3d.h>
#include <string>

#include "Subsystem.hpp"
#include "LimeLight.hpp"

using namespace frc;
class Vision:SubsystemBase
{
    public:

    Vision();

    ~Vision(){};
    /**
     * Initialization code for disabled mode should go here.
     */
    void DisabledInit() {}

    /**
     * Initialization code for autonomous mode should go here.
     */
    void AutonomousInit() {}


    /**
     * Periodic code for all modes should go here.
     */
    void RobotPeriodic();

    /**
     * Periodic code for disabled mode should go here.
     */
    void DisabledPeriodic() {RobotPeriodic();}

    /**
     * Periodic code for autonomous mode should go here.
     */
    void AutonomousPeriodic() {RobotPeriodic();}


    bool HasTarget();
    double GetTx();
    double GetTy();
    double GetTa();
    int GetTagID();
    Pose3d GetRobotPose();
    double GetDistance();
    double autoRotate();
    double autoTranslateX();
    double autonTranslateY();
    private:

    const double CAMERA_HEIGHTm = 1;
    const double REEF_TARGETm = 0.22;
    const double CAMERA_PITCH = 0;

    const std::string limelightName = "limelight";

    PIDController rotatePid;
    PIDController xPid;
    PIDController yPid;
    AprilTagFieldLayout APRILTAGFIELDLAYOUT;
    
    class LimeLightIO
    {
        public:
        double pitch = 0;
        double yaw = 0;
        double distance= 0;
    };

    LimeLightIO limeLightIO{};
};