#include "subsystems/Drivetrain.hpp"
#include "Constants.hpp"
#include <rev/config/SparkMaxConfig.h>
#include <units/velocity.h>
using namespace rev::spark;
Drivetrain::Drivetrain():leftLeader(FLMotorID, SparkLowLevel::MotorType::kBrushless),
leftFollower(BLMotorID, SparkLowLevel::MotorType::kBrushless),
rightLeader(FRMotorID, SparkLowLevel::MotorType::kBrushless),
rightFollwer(BRMotorID, SparkLowLevel::MotorType::kBrushless),
leftEncoder(leftLeader.GetEncoder()),
rightEncoder(rightLeader.GetEncoder()),
leftPIDController(kP, kI, kD),
rightPIDController(kP, kI, kD),
gyro(studica::AHRS::NavXComType::kMXP_SPI, studica::AHRS::NavXUpdateRate::k50Hz),
kinematics((units::meter_t)(kTrackWidth)),
odometry(gyro.GetRotation2d(), (units::meter_t)leftEncoder.GetPosition(),(units::meter_t)rightEncoder.GetPosition() ),
leftMotorFeedforward(kS, kV, kA, dt),
rightMotorFeedforward(kS, kV, kA, dt)
{
    SparkMax t(1, rev::spark::SparkLowLevel::MotorType::kBrushless);
    gyro.Reset();
    SparkMaxConfig lfconfig{}, lbconfig{}, rfconfig{}, rbconfig{};
    periodicIO.diffWheelSpeeds.left = (units::meters_per_second_t)0;
    periodicIO.diffWheelSpeeds.right = (units::meters_per_second_t)0;
    lfconfig
        .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kCoast);
    lfconfig.encoder
        .PositionConversionFactor(kMetersPerRev)
        .VelocityConversionFactor(kMetersPerRev / 60);
    leftLeader.Configure(lfconfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
    
    lbconfig
        .SetIdleMode(SparkMaxConfig::IdleMode::kCoast)
        .Follow(leftLeader, false);
    leftFollower.Configure(lbconfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);

    rfconfig
        .Inverted(true)
        .SetIdleMode(SparkMaxConfig::IdleMode::kCoast);
    rfconfig.encoder
        .PositionConversionFactor(kMetersPerRev)
        .VelocityConversionFactor(kMetersPerRev / 60);
    rightLeader.Configure(rfconfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
    
    rbconfig
        //.Inverted(true)
        .SetIdleMode(SparkMaxConfig::IdleMode::kCoast)
        .Follow(rightLeader, false);
    rightFollwer.Configure(rbconfig,SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
    


    leftEncoder.SetPosition(0.0);
    rightEncoder.SetPosition(0.0);
}

Drivetrain::~Drivetrain()
{
    
}

void Drivetrain:: Drive(double xSpeed, double rot) 
{
    
      periodicIO.diffWheelSpeeds = kinematics
          .ToWheelSpeeds(ChassisSpeeds((units::meters_per_second_t)(xSpeed * kSpeedModeScale), (units::meters_per_second_t)0, (units::radians_per_second_t)(rot * kSlowModeRotScale)));
}

void Drivetrain::RobotPeriodic() 
{
    auto leftFeedforward = leftMotorFeedforward.Calculate(periodicIO.diffWheelSpeeds.left);
    auto rightFeedforward = rightMotorFeedforward.Calculate(periodicIO.diffWheelSpeeds.right);
    double leftOutput = leftPIDController.Calculate(leftEncoder.GetVelocity(),
        (double)periodicIO.diffWheelSpeeds.left);
    double rightOutput = rightPIDController.Calculate(rightEncoder.GetVelocity(),
        (double)periodicIO.diffWheelSpeeds.right);

    periodicIO.leftVoltage = leftOutput + (double)leftFeedforward;
    periodicIO.rightVoltage = rightOutput + (double)rightFeedforward;

    updateOdometry();
}

void Drivetrain:: Drive(ChassisSpeeds speeds) 
{
    periodicIO.diffWheelSpeeds = kinematics.ToWheelSpeeds(speeds);
}

void Drivetrain::clearTurnPIDAccumulation() 
{
    leftPIDController.Reset();
    rightPIDController.Reset();
}

void Drivetrain::setGyroAngleAdjustment(double angle) 
{
    gyro.SetAngleAdjustment(angle);
}

  /** Update robot odometry. */
void Drivetrain::updateOdometry() 
{
    odometry.Update(gyro.GetRotation2d(), (units::meter_t)leftEncoder.GetPosition(), (units::meter_t)rightEncoder.GetPosition());
}

  /** Resets robot odometry. */
void Drivetrain::resetOdometry(Pose2d pose)
{

    odometry.ResetPosition(
        gyro.GetRotation2d(),
        (units::meter_t)(leftEncoder.GetPosition()),
        (units::meter_t)rightEncoder.GetPosition(),
        pose);
}