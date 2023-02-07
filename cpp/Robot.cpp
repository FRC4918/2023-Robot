// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <photonlib/PhotonUtils.h>
#include <photonlib/PhotonCamera.h>
#include <units/angle.h>
#include <units/length.h>

#include "Drivetrain.h"

double speedfactor;
int pipeIndex = 0;

class Robot : public frc::TimedRobot
{
public:
   void RobotInit() override
   {
      // wpi::PortForwarder::GetInstance().Add(5800, "photonvision.local", 5800);
      // PF for PhotonVision
      frc::SmartDashboard::PutNumber("Pipeline", pipeIndex);
   }

   void RobotPeriodic() override
   {
      if (m_controller.GetBackButton())
      {
         m_swerve.Reset();
      }
      if (m_controller.GetBButton())
      {
         speedfactor = 0.2;
      }
      else
      {
         speedfactor = 1.0;
      }

      // Get all m_controller inputs

      // if
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

   void MainDrive(bool fieldRelative)
   {
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
                             Drivetrain::kMaxSpeed *
                             speedfactor;

         // Get the y speed or sideways/strafe speed. We are inverting this because
         // we want a positive value when we pull to the left. Xbox controllers
         // return positive values when you pull to the right by default.
         const auto ySpeed = -m_yspeedLimiter.Calculate(
                                 frc::ApplyDeadband(m_controller.GetLeftX(), 0.10)) *
                             Drivetrain::kMaxSpeed *
                             speedfactor;

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

         const units::meter_t CAMERA_HEIGHT = 13_in;
         const units::meter_t TARGET_HEIGHT = 1_in;
         // Change me later ^^^
         const units::radian_t CAMERA_PITCH = 0_deg;
         const units::meter_t GOAL_RANGE_METERS = 2_ft;

         photonlib::PhotonCamera camera("main");

         // Change pipeline
         pipeIndex = frc::SmartDashboard::GetNumber("Pipeline", 0);
         camera.SetPipelineIndex(pipeIndex);

         double forwardSpeed;
         double sidewaySpeed;
         double rotationSpeed;
         const auto &result = camera.GetLatestResult();

         if (result.HasTargets())
         {
            // First calculate range
            units::meter_t range =
                photonlib::PhotonUtils::CalculateDistanceToTarget(
                    CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH,
                    units::degree_t{result.GetBestTarget().GetPitch()});
            // std::cout << "Range: " << range.to<double>() << std::endl;

            // Use this range as the measurement we give to the PID controller.
            // forwardSpeed = GOAL_RANGE_METERS.to<double>();
            forwardSpeed = (range.to<double>() - 0.6);

            // Calculate strafe
            // sidewaySpeed = result.GetBestTarget().GetX();
            // Something like this ^^

            // Also calculate angular power
            rotationSpeed = result.GetBestTarget().GetYaw();
         }
         else
         {
            // If we have no targets, stay still.
            forwardSpeed = 0;
            sidewaySpeed = 0;
            rotationSpeed = 0;
         }

         // forwardSpeed = forwardSpeed / 10;
         // if (forwardSpeed > 0.5) {
         //   forwardSpeed = 0;
         //}
         // else if (forwardSpeed < -0.5)
         //{
         //   forwardSpeed = 0;
         //}
         // forwardSpeed = (result.GetBestTarget().GetArea() * -1 + 1.5) / 3;
         sidewaySpeed = sidewaySpeed / 10;
         rotationSpeed = rotationSpeed / 40;

         std::cout << "Forward speed: " << forwardSpeed << " Rotation speed: " << rotationSpeed << std::endl;

         const auto xSpeed = m_xspeedLimiter.Calculate(
                                 frc::ApplyDeadband(forwardSpeed, 0.30)) *
                             Drivetrain::kMaxSpeed;

         // const auto ySpeed = -m_yspeedLimiter.Calculate(
         //                         frc::ApplyDeadband(sidewaySpeed, 0.01)) *
         //                     Drivetrain::kMaxSpeed;

         const auto ySpeed = -m_yspeedLimiter.Calculate(
                                 frc::ApplyDeadband(m_controller.GetLeftX(), 0.10)) *
                             Drivetrain::kMaxSpeed;

         const auto rot = -m_rotLimiter.Calculate(
                              frc::ApplyDeadband(rotationSpeed, 0.01)) *
                          Drivetrain::kMaxAngularSpeed;
         m_swerve.Drive(xSpeed, ySpeed, rot, false);
      }
   }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
   return frc::StartRobot<Robot>();
}
#endif
