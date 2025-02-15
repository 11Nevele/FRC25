#pragma once

#include <units/length.h>
#include <units/voltage.h>
#include <units/velocity.h>

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Pose2d.h>

#include "ctre/Phoenix.h"

#include "studica/AHRS.h"

#include "subsystems/Subsystem.hpp"
#include "Robotconfig.hpp"

using namespace frc;
using namespace ctre::phoenix6;

class Drivetrain : SubsystemBase
{

  public:
  
  const double kMaxSpeed = 2.0;
  const double kMaxBoostSpeed = 4.0;
 // 3 meters per second.
  const double kMaxAcceleration = 8.0;

  // 0.7 rotations per second.
  const double kMaxAngularSpeed = M_PIl;


        /**
     * Constructs a SubsystemBase.
     */
  Drivetrain();

    /**
     * Copy constructor.
     */
  Drivetrain(const Drivetrain&) = default;

    /**
     * Copy assignment operator.
     */
  Drivetrain& operator=(const Drivetrain&) = default;

    /**
     * Move constructor.
     */
  Drivetrain(Drivetrain&&) = default;

    /**
     * Move assignment operator.
     */
  Drivetrain& operator=(Drivetrain&&) = default;

  ~Drivetrain();

  void Drive(double, double);

  void Drive(ChassisSpeeds);

  void clearTurnPIDAccumulation();

  void setGyroAngleAdjustment(double);

  /** Update robot odometry. */
  void updateOdometry();

  /** Resets robot odometry. */
  void resetOdometry(Pose2d pose);


    /**
     * Initialization code for disabled mode should go here.
     */
  void DisabledInit(){};

    /**
     * Initialization code for autonomous mode should go here.
     */
  void AutonomousInit(){};


    /**
     * Periodic code for all modes should go here.
     */
  void RobotPeriodic();

    /**
     * Periodic code for disabled mode should go here.
     */
  void DisabledPeriodic(){};

    /**
     * Periodic code for autonomous mode should go here.
     */
  void AutonomousPeriodic(){};

  private:

  const double kSlowModeRotScale = 0.1;
  const double kSpeedModeScale = 2.0;
  const double kTippyModeScale = 0.7;

  const double kTrackWidth =  10;//units::convert<units::length::inch, units::length::meter>(20.75);
  const double kWheelRadius = 10;//units::convert<units::length::inch, units::length::meter>(3.0);
  const double kGearRatio = 10.71;
  const double kMetersPerRev = (2.0 * M_PIl * kWheelRadius) / kGearRatio;

  hardware::TalonFX leftLeader, leftFollower, rightLeader, rightFollwer;
  rev::spark::SparkRelativeEncoder leftEncoder, rightEncoder;

  frc::PIDController leftPIDController, rightPIDController;

  studica::AHRS gyro;

  frc::DifferentialDriveKinematics kinematics;
  frc::DifferentialDriveOdometry odometry;

  bool isNodePoseSet = false;

  frc::SimpleMotorFeedforward<units::meters> leftMotorFeedforward, rightMotorFeedforward; 

  struct PeriodicIO
  {
    DifferentialDriveWheelSpeeds diffWheelSpeeds;
    double leftVoltage = 0.0;
    double rightVoltage = 0.0;
  };
  PeriodicIO periodicIO;


};