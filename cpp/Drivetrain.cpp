// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"
#include <iostream>

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative) {
  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  if (iCallCount % 50 == 0) {
//      auto [distance0, angle0] = m_frontLeft.GetPosition();
//      auto [distance1, angle1] = m_frontRight.GetPosition();
//      auto [distance2, angle2] = m_backRight.GetPosition();
//      auto [distance3, angle3] = m_backLeft.GetPosition();

//      std::cout << "FrontLeft: " << distance0.value() << ", " <<
//                                    angle0.Degrees().value() << std::endl;
//      std::cout << "FrontRight: " << distance1.value() << ", " <<
//                                     angle1.Degrees().value() << std::endl;
//      std::cout << "BackRight: " << distance2.value() << ", " <<
//                                    angle2.Degrees().value() << std::endl;
//      std::cout << "BackLeft: " << distance3.value() << ", " <<
//                                   angle3.Degrees().value() << std::endl;

//    std::cout << "Current Gyro Position: " << m_gyro.GetAngle() << std::endl;
  }
  iCallCount++;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void Drivetrain::Reset() {
	m_gyro.Reset();
}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_backLeft.GetPosition(), m_backRight.GetPosition()});
}