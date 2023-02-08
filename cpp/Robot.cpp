// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <iostream>

#include <cstdio>
#include <span>
#include <sstream>
#include <string>
#include <thread>
#include <cameraserver/CameraServer.h>
#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <frc/geometry/Transform3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <units/angle.h>
#include <units/length.h>

// The block above is for AprilTag

#include "Drivetrain.h"

double xid3;
double zid3;
double rotationid3;
int speedfactor;

class Robot : public frc::TimedRobot
{
public:
   void RobotInit() override
   {
      std::thread visionThread(VisionThread);
      visionThread.detach();
   }

   void RobotPeriodic() override
   {
      if (m_controllerdrive.GetBackButton())
      {
         m_swerve.Reset();
      }
      if (m_controllerdrive.GetBButton())
      {
         speedfactor = 0.2;
      }
      else
      {
         speedfactor = 1.0;
      }
   }

   void AutonomousPeriodic() override
   {
      DriveWithJoystick(false);
      m_swerve.UpdateOdometry();
   }

   // change to true for field relative
   //          false for robot relative
   void TeleopPeriodic() override
   {
      DriveWithJoystick(true);
   }

private:

   // Controller 0 for driver, 1 for operator
   // Knock-off will be controller 1
   frc::XboxController m_controllerdrive{0};
   frc::XboxController m_controllerop{1};

   Drivetrain m_swerve;

   // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
   // to 1.
   // was: frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{10 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{10 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_rotLimiter{10 / 1_s};

   void DriveWithJoystick(bool fieldRelative)

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
      if (!m_controllerdrive.GetAButton())
      {
         const auto xSpeed = -m_xspeedLimiter.Calculate(
                                 frc::ApplyDeadband(m_controllerdrive.GetLeftY(), 0.10)) *
                             Drivetrain::kMaxSpeed *
                             speedfactor;

         // Get the y speed or sideways/strafe speed. We are inverting this because
         // we want a positive value when we pull to the left. Xbox controllers
         // return positive values when you pull to the right by default.
         const auto ySpeed = -m_yspeedLimiter.Calculate(
                                 frc::ApplyDeadband(m_controllerdrive.GetLeftX(), 0.10)) *
                             Drivetrain::kMaxSpeed *
                             speedfactor;

         // Get the rate of angular rotation. We are inverting this because we want a
         // positive value when we pull to the left (remember, CCW is positive in
         // mathematics). Xbox controllers return positive values when you pull to
         // the right by default.
         const auto rot = -m_rotLimiter.Calculate(
                              frc::ApplyDeadband(m_controllerdrive.GetRightX(), 0.10)) *
                          Drivetrain::kMaxAngularSpeed;

         m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);
      }
      else // A button
      {
         // Get last vison target of tag 3
         const auto xSpeed = m_xspeedLimiter.Calculate(
                                 frc::ApplyDeadband(zid3, 0.01)) *
                             Drivetrain::kMaxSpeed;

         const auto ySpeed = -m_yspeedLimiter.Calculate(
                                 frc::ApplyDeadband(xid3, 0.01)) *
                             Drivetrain::kMaxSpeed;

         // Deadband high to disable
         const auto rot = -m_rotLimiter.Calculate(
                              frc::ApplyDeadband(rotationid3, 1000.00)) *
                          Drivetrain::kMaxAngularSpeed;

         m_swerve.Drive(xSpeed, ySpeed, rot, false);
      }
   }

private:
   static void VisionThread()
   {
      frc::AprilTagDetector detector;
      // Looks for tag16h5, don't correct any error bits
      detector.AddFamily("tag16h5", 0);

      // Parameters are for a Microsoft Lifecam HD-3000
      frc::AprilTagPoseEstimator::Config poseEstConfig = {
          .tagSize = units::length::inch_t(6.0),
          .fx = 699.3778103158814,
          .fy = 677.7161226393544,
          .cx = 345.6059345433618,
          .cy = 207.12741326228522};
      frc::AprilTagPoseEstimator estimator =
          frc::AprilTagPoseEstimator(poseEstConfig);

      // Get the USB camera from CameraServer
      cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
      // Set the resolution
      camera.SetResolution(640, 480);
      camera.SetExposureManual(25);

      // Get a CvSink. This will capture Mats from the Camera
      cs::CvSink cvSink = frc::CameraServer::GetVideo();
      // Setup a CvSource. This will send images back to the Dashboard
      cs::CvSource outputStream =
          frc::CameraServer::PutVideo("Detected", 640, 480);

      // Mats are very memory expensive. Lets reuse this Mat.
      cv::Mat mat;
      cv::Mat grayMat;

      // Instantiate once
      std::vector<int> tags;
      cv::Scalar outlineColor = cv::Scalar(0, 255, 0);
      cv::Scalar crossColor = cv::Scalar(0, 0, 255);

      while (true)
      {
         // Tell the CvSink to grab a frame from the camera and
         // put it
         // in the source mat.  If there is an error notify the
         // output.
         if (cvSink.GrabFrame(mat) == 0)
         {
            // Send the output the error.
            outputStream.NotifyError(cvSink.GetError());
            // skip the rest of the current iteration
            continue;
         }

         cv::cvtColor(mat, grayMat, cv::COLOR_BGR2GRAY);

         cv::Size g_size = grayMat.size();
         frc::AprilTagDetector::Results detections =
             detector.Detect(g_size.width, g_size.height, grayMat.data);

         // have not seen any tags yet
         tags.clear();

         for (const frc::AprilTagDetection *detection : detections)
         {
            // remember we saw this tag
            tags.push_back(detection->GetId());

            // draw lines around the tag
            for (int i = 0; i <= 3; i++)
            {
               int j = (i + 1) % 4;
               const frc::AprilTagDetection::Point pti = detection->GetCorner(i);
               const frc::AprilTagDetection::Point ptj = detection->GetCorner(j);
               line(mat, cv::Point(pti.x, pti.y), cv::Point(ptj.x, ptj.y),
                    outlineColor, 2);
            }

            // mark the center of the tag
            const frc::AprilTagDetection::Point c = detection->GetCenter();
            int ll = 10;
            line(mat, cv::Point(c.x - ll, c.y), cv::Point(c.x + ll, c.y),
                 crossColor, 2);
            line(mat, cv::Point(c.x, c.y - ll), cv::Point(c.x, c.y + ll),
                 crossColor, 2);

            // identify the tag
            putText(mat, std::to_string(detection->GetId()),
                    cv::Point(c.x + ll, c.y), cv::FONT_HERSHEY_SIMPLEX, 1,
                    crossColor, 3);

            // determine pose
            frc::Transform3d pose = estimator.Estimate(*detection);

            // put pose into dashbaord
            std::stringstream dashboardString;
            dashboardString << "Translation: " << units::length::to_string(pose.X())
                            << ", " << units::length::to_string(pose.Y()) << ", "
                            << units::length::to_string(pose.Z());
            frc::Rotation3d rotation = pose.Rotation();
            dashboardString << "; Rotation: "
                            << units::angle::to_string(rotation.X()) << ", "
                            << units::angle::to_string(rotation.Y()) << ", "
                            << units::angle::to_string(rotation.Z());
            frc::SmartDashboard::PutString(
                "pose_" + std::to_string(detection->GetId()),
                dashboardString.str());

            // Get location and rotation of tracker ID3 put in variables (temp)
            if (detection->GetId() == 3)
            {
               // xid3 = pose.X();
               // zid3 = pose.Z();
               // rotationid3 = rotation.Y();

               // Covert units::length::meter_t to double
               xid3 = units::length::meter_t(pose.X()).to<double>();
               zid3 = units::length::meter_t(pose.Z()).to<double>();

               // Convert units::angle::radian_t to degrees in double
               rotationid3 = units::angle::degree_t(rotation.Y()).to<double>();

               // Offset (disatnce from tracker m)
               zid3 = zid3 - 0.75;

               // Divide to make slower
               xid3 = xid3 / 1;
               zid3 = zid3 / 2;

               // rotationid3 = rotationid3 / 3;

               // Max +-
               if (xid3 > 0.2)
               {
                  xid3 = 0.2;
               }
               if (xid3 < -0.2)
               {
                  xid3 = -0.2;
               }
               if (zid3 > 0.2)
               {
                  zid3 = 0.2;
               }
               if (zid3 < -0.2)
               {
                  zid3 = -0.2;
               }

               // Cout

               std::cout << "Sideways: " << xid3 << " Depth: " << zid3 << " Rotation: " << rotationid3 << std::endl;
            }
         }

         // put list of tags onto dashboard
         std::stringstream tags_s;
         if (tags.size() > 0)
         {
            if (tags.size() > 1)
            {
               std::copy(tags.begin(), tags.end() - 1,
                         std::ostream_iterator<int>(tags_s, ","));
            }
            tags_s << tags.back();
         }
         frc::SmartDashboard::PutString("tags", tags_s.str());

         // Give the output stream a new image to display
         outputStream.PutFrame(mat);
      }
   }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
   return frc::StartRobot<Robot>();
}
#endif
