// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <ctre/phoenix/sensors/WPI_PigeonIMU.h>

#include "SwerveModule.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain
{
public:
   Drivetrain() { m_gyro.Reset(); }

   void Reset(void);

   void Drive(units::meters_per_second_t xSpeed,
              units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
              bool fieldRelative);
   void UpdateOdometry();

   static constexpr units::meters_per_second_t kMaxSpeed =
       3.0_mps; // 3 meters per second TODO: check this value
   static constexpr units::radians_per_second_t kMaxAngularSpeed{
       std::numbers::pi}; // 1/2 rotation per second TODO: check this value

private:
   frc::Translation2d m_frontLeftLocation{+0.26_m, +0.26_m};
   frc::Translation2d m_frontRightLocation{+0.26_m, -0.26_m};
   frc::Translation2d m_backLeftLocation{-0.26_m, +0.26_m};
   frc::Translation2d m_backRightLocation{-0.26_m, -0.26_m};

   SwerveModule m_frontLeft{8, 9, 0, 36};
   SwerveModule m_frontRight{10, 11, 1, 34};
   SwerveModule m_backLeft{14, 15, 3, 24};
   SwerveModule m_backRight{12, 13, 2, 150};

   // frc::AnalogGyro m_gyro{0};
   ctre::phoenix::sensors::WPI_PigeonIMU m_gyro{1};

   frc::SwerveDriveKinematics<4> m_kinematics{
       m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
       m_backRightLocation};

   frc::SwerveDriveOdometry<4> m_odometry{
       m_kinematics,
       m_gyro.GetRotation2d(),
       {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
        m_backLeft.GetPosition(), m_backRight.GetPosition()}};

   int iCallCount = 0;
};
