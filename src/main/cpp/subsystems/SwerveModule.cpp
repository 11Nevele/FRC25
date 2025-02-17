// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

#include "Constants.hpp"
using namespace units;
SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
                           const int driveEncoderPorts,
                           const int turningEncoderPorts,
                           bool driveReversed,
                           bool turnReversed,
                           double canCoderOffset)
    : m_driveMotor(driveMotorChannel),
      m_turningMotor(turningMotorChannel),
      m_turningEncoder(turningEncoderPorts) 
{
    configs::CANcoderConfiguration canCoderConfig;
    canCoderConfig.MagnetSensor.MagnetOffset = turn_t{canCoderOffset};
    canCoderConfig.MagnetSensor.SensorDirection = turnReversed;
    m_turningEncoder.GetConfigurator().Apply(canCoderConfig);
    
    configs::TalonFXConfiguration driveConfig;
    driveConfig.Slot0.kP = ModuleConstants::kPDrive;
    driveConfig.Slot0.kI = ModuleConstants::kIDrive;
    driveConfig.Slot0.kD = ModuleConstants::kDDrive;
    driveConfig.Slot0.kV = ModuleConstants::kFDrive;
    driveConfig.MotorOutput.Inverted = driveReversed;
    m_driveMotor.GetConfigurator().Apply(driveConfig);
    
    configs::TalonFXConfiguration turningConfig;
    turningConfig.Slot0.kP = ModuleConstants::kPTurning;
    turningConfig.Slot0.kI = ModuleConstants::kITurning;
    turningConfig.Slot0.kD = ModuleConstants::kDTurning;
    turningConfig.Slot0.kV = ModuleConstants::kFTurning;
    turningConfig.MotorOutput.Inverted = turnReversed;
    turningConfig.Feedback.FeedbackRemoteSensorID = m_turningEncoder.GetDeviceID();
    
    turningConfig.MotionMagic.MotionMagicAcceleration = turns_per_second_squared_t{4096};
    turningConfig.MotionMagic.MotionMagicCruiseVelocity = turns_per_second_t{5108} ;
    m_turningMotor.GetConfigurator().Apply(turningConfig);

}

frc::SwerveModuleState SwerveModule::GetState() 
{
  return {meters_per_second_t{GetDriveEncoderVelocity()},
        degree_t{getTurningEncoderAngle()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() 
{
  return {meter_t{GetDriveEncoderPosition()},
          degree_t{getTurningEncoderAngle()}
          };
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState& referenceState) 
{
  desiredState = referenceState;
  frc::Rotation2d encoderRotation
  {
      degree_t{getTurningEncoderAngle()}
  };


  // Optimize the reference state to avoid spinning further than 90 degrees
  desiredState.Optimize(encoderRotation);

  // Scale speed by cosine of angle error. This scales down movement
  // perpendicular to the desired direction of travel that can occur when
  // modules change directions. This results in smoother driving.
  desiredState.CosineScale(encoderRotation);

  controls::VelocityVoltage v{0_tps};
  v.Slot = 0;
  // Set the motor outputs.
  m_driveMotor.SetControl(v.WithVelocity(turns_per_second_t{desiredState.speed.value() / ModuleConstants::kDriveCoefficient / 10.0}));

    controls::MotionMagicVoltage m_motmag{0_tr};
    m_motmag.Slot = 0;
  // 將角度轉換成 raw unit, 並設給 turning
    m_turningMotor.SetControl(m_motmag.WithPosition(turn_t{desiredState.angle.Degrees().value() / 360.0}));
}

void SwerveModule::ResetEncoders() 
{
  m_driveMotor.SetPosition(turn_t{0});
  m_turningMotor.SetPosition(turn_t{0});
}
