#include "subsystems/DriveSubsystem.h"

DriveSubsystem::DriveSubsystem():
m_frontLeft(0, 1, 0, 1, false, false, 0),
m_rearLeft(2, 3, 2, 3, false, false, 0),
m_frontRight(4, 5, 4, 5, false, false,0),
m_rearRight(6, 7, 6, 7, false, false,0),
m_odometry{kDriveKinematics,
                 m_gyro.GetRotation2d(),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}}
{

}

void DriveSubsystem::Drive(double xSpeed, double ySpeed, double rot)
{
    ChassisSpeeds newSpeed;
    newSpeed.vx = (units::meters_per_second_t)xSpeed;
    newSpeed.vy = (units::meters_per_second_t)ySpeed;
    newSpeed.omega = (units::radians_per_second_t)rot;
    SetChassisSpeed(newSpeed);
}

void DriveSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates)
{
    m_frontLeft.SetDesiredState(desiredStates[0]);
    m_rearLeft.SetDesiredState(desiredStates[1]);
    m_frontRight.SetDesiredState(desiredStates[2]);
    m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::SetChassisSpeed(ChassisSpeeds speed)
{
    wpi::array<frc::SwerveModuleState, 4> newStates = kDriveKinematics.ToSwerveModuleStates(speed);
    SetModuleStates(newStates);

}