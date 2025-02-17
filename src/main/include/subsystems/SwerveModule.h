// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/trajectory/TrapezoidProfile.h>


#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/configs/Configs.hpp"
#include "Constants.hpp"

using namespace ctre::phoenix6;
using namespace frc;



class SwerveModule 
{
 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel,
               const int driveEncoderPorts, const int turningEncoderPorts,
               bool driveEncoderReversed, bool turningEncoderReversed, double offset);
    //in meters
    double GetDriveEncoderPosition() 
    {
        return m_driveMotor.GetPosition().GetValueAsDouble() * ModuleConstants::kEncoderCPR; // m
    }
    double GetDriveEncoderVelocity() 
    {
        return m_driveMotor.GetVelocity().GetValueAsDouble() * ModuleConstants::kEncoderCPR * 10; // m/s
    }

    //in degrees
    double getTurningEncoderAngle() 
    {
        return m_turningMotor.GetPosition().GetValueAsDouble() / 4096.0 * 360.0; // deg
    }


  frc::SwerveModuleState GetState();

  frc::SwerveModulePosition GetPosition();

  void SetDesiredState(frc::SwerveModuleState& state);

  void ResetEncoders();

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.
    double deg2raw(double deg) 
    {
        return deg / 360 * 4096;
    }
  static constexpr auto kModuleMaxAngularVelocity =
      units::radians_per_second_t{std::numbers::pi};
  static constexpr auto kModuleMaxAngularAcceleration =
      units::radians_per_second_squared_t{std::numbers::pi * 2.0};

  hardware::TalonFX m_driveMotor;
  hardware::TalonFX m_turningMotor;

  hardware::CANcoder m_turningEncoder;

  SwerveModuleState desiredState;

};
