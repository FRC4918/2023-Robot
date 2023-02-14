// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <units/angle.h>
#include <units/length.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include "Drivetrain.h"

double speedfactor;
int pipelineindex = 0;
bool ldriver = 0;

// nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("camerapose_targetspace",std::vector<double>(6));

class Robot : public frc::TimedRobot
{
public:
   void RobotInit() override
   {
   }

   void RobotPeriodic() override
   {
      // I really do not like this, but it works for now.
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

      if (m_controller.GetLeftBumperPressed())
      {
         pipelineindex--;
         if (pipelineindex < 0)
         {
            pipelineindex = 3;
         }
      }
      if (m_controller.GetRightBumperPressed())
      {
         pipelineindex++;
         if (pipelineindex > 3)
         {
            pipelineindex = 0;
         }
      }
      if (m_controller.GetYButtonPressed())
      {
         ldriver = !ldriver;
      }
      std::shared_ptr<nt::NetworkTable> ltable =

         nt::NetworkTableInstance::GetDefault().GetTable("limelight");

      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline",pipelineindex);
      // 0 = Drivermode, lights off
      // 1 = Visionmode, lights auto
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode",ldriver);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",ldriver);
      

      std::vector<double> lcam_pose_target = ltable->GetNumberArray("tid", std::vector<double>(6));
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
         double forwardSpeed;
         double sidewaySpeed;
         double rotationSpeed;

         const auto xSpeed = m_xspeedLimiter.Calculate(
                                 frc::ApplyDeadband(forwardSpeed, 0.30)) *
                             Drivetrain::kMaxSpeed;

         const auto ySpeed = -m_yspeedLimiter.Calculate(
                                 frc::ApplyDeadband(sidewaySpeed, 0.01)) *
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
