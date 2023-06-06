// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

#include "Constants.h"

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel)
    : m_driveMotor(driveMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel,
                     rev::CANSparkMax::MotorType::kBrushless),
      m_driveEncoder(m_driveMotor.GetEncoder(
          rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42)),
      m_turningEncoder(turningEncoderChannel)
{
  m_driveEncoder.SetPositionConversionFactor(
      2 * std::numbers::pi * ModuleConstants::kWheelRadius / ModuleConstants::kDriveGearRatio); // Conversion from rot to m
  m_driveEncoder.SetVelocityConversionFactor(
      (2 * std::numbers::pi * ModuleConstants::kWheelRadius / ModuleConstants::kDriveGearRatio) /
      60); // Converstion from rpm to to m/s

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(-units::radian_t{std::numbers::pi},
                                               units::radian_t{std::numbers::pi});
  //  m_driveMotor.SetOpenLoopRampRate(0.1);
  m_driveMotor.SetSmartCurrentLimit(40);
  m_turningMotor.SetSmartCurrentLimit(40);
}

frc::SwerveModuleState SwerveModule::GetState()
{
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
          units::radian_t{m_turningEncoder.GetAbsolutePosition()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition()
{
  return {units::meter_t{(m_driveEncoder.GetPosition())},
          units::radian_t{m_turningEncoder.GetAbsolutePosition()}};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState &referenceState)
{
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t{(m_turningEncoder.GetPosition())});

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveEncoder.GetVelocity(), state.speed.value());

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t{m_turningEncoder.GetAbsolutePosition()}, state.angle.Radians());

  // Set the motor outputs.
  m_driveMotor.Set(driveOutput);
  m_turningMotor.Set(turnOutput);
}

void SwerveModule::ResetEncoders()
{
  m_driveEncoder.SetPosition(0);
  m_turningEncoder.SetPosition(0);
}