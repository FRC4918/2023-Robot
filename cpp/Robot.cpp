// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <iostream>


#include "Drivetrain.h"

double AUTODRIVEX, AUTODRIVEY, AUTODRIVEROT;

class Robot : public frc::TimedRobot
{
public:
   void RobotInit() override
   {
      // wpi::PortForwarder::GetInstance().Add(5800, "photonvision.local", 5800);
      // PF for PhotonVision
   }

   void RobotPeriodic() override
   {
      if (m_controller.GetBackButton())
      {
         m_swerve.Reset();
      }
   }

   void AutonomousPeriodic() override
   {
      MainDrive(false);
      m_swerve.UpdateOdometry();
   }

   // change to true for field relative
   //          false for robot relative
   void TeleopPeriodic() override
   {
      MainDrive(true);
   }

private:
   frc::XboxController m_controller{0};
   Drivetrain m_swerve;

   // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
   // to 1.
   // was: frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{10 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{10 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_rotLimiter{10 / 1_s};

   void MainDrive(bool fieldRelative) {
      // Get the x speed. We are inverting this because Xbox controllers return
      // negative values when we push forward.
      static int iCallCount = 0;

      iCallCount++;
      if (0 == iCallCount % 50)
      {
         // std::cout << "Joy X/Y/RotX " << m_controller.GetLeftX()  << "/" <<
         //                                 m_controller.GetLeftY()  << "/" <<
         //                                 m_controller.GetRightX() << std::endl;
      }

      // if not A button
      if (!m_controller.GetAButton())
      {
         const auto xSpeed = -m_xspeedLimiter.Calculate(
                                 frc::ApplyDeadband(m_controller.GetLeftY(), 0.10)) *
                             Drivetrain::kMaxSpeed;

         // Get the y speed or sideways/strafe speed. We are inverting this because
         // we want a positive value when we pull to the left. Xbox controllers
         // return positive values when you pull to the right by default.
         const auto ySpeed = -m_yspeedLimiter.Calculate(
                                 frc::ApplyDeadband(m_controller.GetLeftX(), 0.10)) *
                             Drivetrain::kMaxSpeed;

         // Get the rate of angular rotation. We are inverting this because we want a
         // positive value when we pull to the left (remember, CCW is positive in
         // mathematics). Xbox controllers return positive values when you pull to
         // the right by default.
         const auto rot = -m_rotLimiter.Calculate(
                              frc::ApplyDeadband(m_controller.GetRightX(), 0.10)) *
                          Drivetrain::kMaxAngularSpeed;

         m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);
      }
      else // A button
      {

         const auto xSpeed = m_xspeedLimiter.Calculate(
                                 frc::ApplyDeadband(AUTODRIVEX, 0.01)) *
                             Drivetrain::kMaxSpeed;

         const auto ySpeed = -m_yspeedLimiter.Calculate(
                                 frc::ApplyDeadband(AUTODRIVEY, 0.01)) *
                             Drivetrain::kMaxSpeed;

         const auto rot = -m_rotLimiter.Calculate(
                              frc::ApplyDeadband(AUTODRIVEROT, 0.01)) *
                          Drivetrain::kMaxAngularSpeed;

         m_swerve.Drive(xSpeed, ySpeed, rot, true);
      }
   }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
   return frc::StartRobot<Robot>();
}
#endif
