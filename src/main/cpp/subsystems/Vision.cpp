#include "subsystems/vision.hpp"
#include <cmath>


#include "subsystems/vision.hpp"

const auto DegreeToRadian = units::convert<units::degrees, units::radians>;

Vision::Vision() 
    : rotatePid(0.125, 0.0, 0.0), 
      xPid(0.1, 0.0, 0.005), 
      yPid(0.0605, 0.0, 0.0055), 
      APRILTAGFIELDLAYOUT(frc::LoadAprilTagLayoutField(frc::AprilTagField::k2025Reefscape)) 
{
    // Constructor implementation
}

void Vision::RobotPeriodic() 
{
    limeLightIO.pitch = GetTy();
    limeLightIO.yaw = GetTx();
    limeLightIO.distance = GetDistance();
    // Periodic code for all modes
}

bool Vision::HasTarget() 
{
    return LimelightHelpers::getTV();
}

double Vision::GetTx() {
    return LimelightHelpers::getTX();
}

double Vision::GetTy() {
    return LimelightHelpers::getTY();
}

double Vision::GetTa() {
    return LimelightHelpers::getTA();
}

int Vision::GetTagID() {
    return LimelightHelpers::getFiducialID(limelightName);
}

frc::Pose3d Vision::GetRobotPose() 
{
    return LimelightHelpers::toPose3D(LimelightHelpers::getBotpose(limelightName));
}

double Vision::GetDistance() 
{
    double ty = GetTy();
    double angleRadian = DegreeToRadian(limeLightIO.pitch + CAMERA_PITCH);
    if(std::abs(angleRadian) < 1e-6)
        return std::numeric_limits<double>::max();
    return (REEF_TARGETm - CAMERA_HEIGHTm) / std::tan(angleRadian);
}

double Vision::autoRotate() 
{
    double correction = rotatePid.Calculate(GetTx(), 0) * -0.2;
    std::cout << "R: " << correction << std::endl;
    return correction;
}

double Vision::autoTranslateX() 
{
    double correction =
        -xPid.Calculate(GetDistance() * std::sin(DegreeToRadian(GetTx())), 0);
    std::cout << "X: " << correction << std::endl;
    if (HasTarget()) {
      return correction;
    } else {
      return 0;
    }
}

double Vision::autonTranslateY() 
{
    double correction =
        -yPid.Calculate(GetDistance() * std::cos(DegreeToRadian(GetTx())), 10);
    std::cout << "Y: " << correction << std::endl;
    if (HasTarget()) {
      return correction;
    } else {
      return 0;
    }
}