// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#define SAFETY_LIMITS 1

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <iostream>

#include <cstdio>
#include <span>
#include <sstream>
#include <string>
#include <unistd.h>

#include "frc/AnalogInput.h"
#include "frc/BuiltInAccelerometer.h"
#include "frc/PneumaticsModuleType.h"
#include "frc/Compressor.h"
#include "frc/DigitalInput.h"
#include "frc/DigitalOutput.h"
#include "frc/DigitalSource.h"
#include "frc/DoubleSolenoid.h"
#include "frc/DriverStation.h"
#include "frc/Joystick.h"

#include <frc/geometry/Rotation2d.h>

#include "ctre/Phoenix.h"

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

#include <iomanip>
#include <vector>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>


#include "Drivetrain.h"

using std::cout;
using std::endl;
using std::setw;
using std::setfill;         // so we can use "setfill('0') in cout streams
using std::abs;
using namespace cv;

double xid3;
double zid3;
double rotationid3;
int    speedfactor;

                                                            // CTRE compressor
   frc::Compressor m_compressor{ 0, frc::PneumaticsModuleType::CTREPCM };

   frc::DoubleSolenoid m_grabberPortSolenoid{
                               // 0, frc::PneumaticsModuleType::CTREPCM, 4, 6};
                                  0, frc::PneumaticsModuleType::CTREPCM, 0, 2};
   frc::DoubleSolenoid m_grabberStbdSolenoid{
                               // 0, frc::PneumaticsModuleType::CTREPCM, 5, 7};
                                  0, frc::PneumaticsModuleType::CTREPCM, 1, 3};
   WPI_TalonSRX m_ExtenderMotor{  3 };   // motor for arm extender
   WPI_TalonSRX m_WristMotor{    12 };   // motor for arm wrist

class Robot : public frc::TimedRobot
{

   int iCallCount = 0;

private:

   frc::XboxController m_DriveController{0};
   frc::XboxController m_OperatorController{1};
   frc::Joystick       m_Console{3};   // the number 3 USB device
   Drivetrain m_swerve;

   // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
   // to 1.
   // was: frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{10 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{10 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_rotLimiter{10 / 1_s};

   const frc::Rotation2d jagrotzero { (units::degree_t)0.0 };
   // const frc::Pose2d DestinationOne { 13.0, 0.0,  jagrotzero };
   const frc::Pose2d DestinationOne = { (units::foot_t)13.0,
                                        (units::foot_t)0.0,
                                        (units::degree_t)0.0 };

   void MotorInitSpark(rev::CANSparkMax &m_motor);
   static const int ShoulderMotorDeviceID =  16;
//   static const int WristMotorDeviceID =  17;
   rev::CANSparkMax m_ShoulderMotor{ ShoulderMotorDeviceID,
                                     rev::CANSparkMax::MotorType::kBrushless};
//   rev::CANSparkMax m_WristMotor{ WristMotorDeviceID,
//                                  rev::CANSparkMax::MotorType::kBrushless};

   std::shared_ptr<nt::NetworkTable> limenttable =
               nt::NetworkTableInstance::GetDefault().GetTable( "limelight" );

                                            // create a list of maneuver types
   enum MANEUVER_TYPE {
      M_TERMINATE_SEQ  = 0,
      M_STOP           = 1,
      M_GO_TO_POSE     = 2,  // drive straight to a specified pose
      M_WAIT           = 3,  // wait for a period of time (a number of ticks)
      M_BALANCE        = 4   // balance on the charging station (teeter-totter)
   };

                    // create a struct which can contain a full maneuver
                 // (type, desired pose, including yaw (heading), etc.)
   struct maneuver {
      int                index;     // index of this element in an array of
                                    // maneuvers
      enum MANEUVER_TYPE type;      // type of maneuver (stop, turn, etc.)
      frc::Pose2d        DestinationPose;  // pose to drive toward
      double             dArg;      // General-purpose argument (seconds, etc.)
      bool               bArg;      // General-purpose boolean argument
   };

   int mSeqIndex = 0;

                  // Create a sequence of full maneuvers
   struct maneuver mSeq[256] =
   {
     // Perform a maneuver until the specified distance and heading
     // has been achieved (the robot is at the desired pose).
     // Distances are relative to the end of the previous maneuver;
     // headings are absolute, from the initial yaw when AutonomousInit()
     // was called and sCurrState.initialYaw was set.
     //
     //                        destination                    
     //                        pose (meters              double  boolean
     // index command          and degrees)              arg1    arg2  
     // ----- ---------------- ------------------------  ------  -------
      // index 00: simple drive autonomous; forward, right, back, balance
      {   0,  M_GO_TO_POSE,    { (units::foot_t)13.0,  // X (forward)
                                 (units::foot_t)0.0,   // Y (sideways)
                                 (units::degree_t)0.0 },  0.0,    false },
      {   1,  M_GO_TO_POSE,    { (units::foot_t)13.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {   2,  M_GO_TO_POSE,    { (units::foot_t)9.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {   3,  M_BALANCE,       { (units::foot_t)9.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {   4,  M_STOP,          { (units::foot_t)9.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {   5,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {   6,  M_WAIT,          { (units::foot_t)9.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  1.0,    false },
      {   7,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {   8,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {   9,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  0.0,    false },

      // index 10: simple drive autonomous; forward, back, balance
      {  10,  M_GO_TO_POSE,    { (units::foot_t)13.0,
                                 (units::foot_t)0.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  11,  M_GO_TO_POSE,    { (units::foot_t)9.0,
                                 (units::foot_t)0.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  12,  M_BALANCE,       { (units::foot_t)9.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  13,  M_STOP,          { (units::foot_t)9.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  14,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  15,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  16,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  17,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  18,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  19,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                                 (units::foot_t)-6.0,
                                 (units::degree_t)0.0 },  0.0,    false },

      // index 20: simple drive autonomous; forward, left, back, balance
      {  20,  M_GO_TO_POSE,    { (units::foot_t)13.0,
                                 (units::foot_t)0.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  21,  M_GO_TO_POSE,    { (units::foot_t)13.0,
                                 (units::foot_t)6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  22,  M_GO_TO_POSE,    { (units::foot_t)9.0,
                                 (units::foot_t)6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  23,  M_BALANCE,       { (units::foot_t)9.0,
                                 (units::foot_t)6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  24,  M_STOP,          { (units::foot_t)9.0,
                                 (units::foot_t)6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  25,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                                 (units::foot_t)6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  26,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                                 (units::foot_t)6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  27,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                                 (units::foot_t)6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  28,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                                 (units::foot_t)6.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  29,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                                 (units::foot_t)6.0,
                                 (units::degree_t)0.0 },  0.0,    false },

      // index 30: test autonomous; forward, stop, wait 3 seconds, back, stop
      {  30,  M_GO_TO_POSE,    { (units::foot_t)3.0,
                                 (units::foot_t)0.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  31,  M_STOP,          { (units::foot_t)3.0,
                                 (units::foot_t)0.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  32,  M_WAIT,          { (units::foot_t)3.0,     // wait 3 seconds
                                 (units::foot_t)0.0,
                                 (units::degree_t)0.0 },  150.0,  false },
      {  33,  M_GO_TO_POSE,    { (units::foot_t)0.0,
                                 (units::foot_t)0.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  34,  M_STOP,          { (units::foot_t)0.0,
                                 (units::foot_t)0.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  35,  M_TERMINATE_SEQ, { (units::foot_t)0.0,
                                 (units::foot_t)0.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  36,  M_TERMINATE_SEQ, { (units::foot_t)0.0,
                                 (units::foot_t)0.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  37,  M_TERMINATE_SEQ, { (units::foot_t)0.0,
                                 (units::foot_t)0.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  38,  M_TERMINATE_SEQ, { (units::foot_t)0.0,
                                 (units::foot_t)0.0,
                                 (units::degree_t)0.0 },  0.0,    false },
      {  39,  M_TERMINATE_SEQ, { (units::foot_t)0.0,
                                 (units::foot_t)0.0,
                                 (units::degree_t)0.0 },  0.0,    false },

      {  40,  M_TERMINATE_SEQ, { (units::foot_t)0.0,
                                 (units::foot_t)0.0,
                                 (units::degree_t)0.0 },  0.0,    false },
   };   // struct maneuver mSeq[256]

   struct sState {
      double joyX;
      double joyY;
      double joyZ;
      double conX;
      double conY;
      bool   joyButton[12];
      bool   conButton[13];
      double dLSMasterPosition;
      double dRSMasterPosition;
      int    iTSMasterPosition; // Top Shooter
      int    iBSMasterPosition; // Bottom Shooter
      double dLSMasterVelocity;
      double dRSMasterVelocity;
      int    iTSMasterVelocity; // Top Shooter
      int    iBSMasterVelocity; // Bottom Shooter
      int    iIntakePercent;
      double yawPitchRoll[3];  // position data from Pigeon or ADIS16470 IMU
      double rateXYZ[3];       // rate data from Pigeon IMU or ADIS16470 IMU;
                               // index [2] (Z axis) is yaw rate in deg/sec.
      double yawPosnEstimate;  // Current estimate of yaw position in degrees
                               // (left-positive).  This is necessary because
                               // the gyro only updates the yaw position once
                               // every 0.1 second (5 20-millisecond ticks).
      double yawRateEstimate;  // Current estimate of yaw turn rate in deg/sec
                               // (left-positive).  Necessary because the gyro
                               // only updates every 0.1 second (5 ticks).
      frc::Pose2d CurrentRobotPose;
      frc::Pose2d DesiredRobotPose;
      double initialYaw;
      bool   teleop;
      double dLimelightDistanceToGoal;
      double dLimelightDesiredShooterSpeed;
   } sCurrState, sPrevState;

      // Console button  1 (the left-top pushbutton switch on the console)
      // runs the climber up.
#define BUTTON_BLUECLIMBERUP            ( sCurrState.conButton[3] )
#define BUTTON_BLUECLIMBERUP_PREV       ( sPrevState.conButton[3] )
#define BUTTON_REDCLIMBERUP            ( sCurrState.conButton[4] )
#define BUTTON_REDCLIMBERUP_PREV       ( sPrevState.conButton[4] )
      // Console button  5 (the center-top pushbutton switch on the console)
      // runs the conveyor forward.
#define BUTTON_CONVEYORFORWARD      ( sCurrState.conButton[5] )
#define BUTTON_CONVEYORFORWARD_PREV ( sPrevState.conButton[5] )
      // Console button  3 (the right-top pushbutton switch on the console)
      // runs the climber down.
#define BUTTON_BLUECLIMBERDOWN          ( sCurrState.conButton[1] )
#define BUTTON_BLUECLIMBERDOWN_PREV     ( sPrevState.conButton[1] )
#define BUTTON_REDCLIMBERDOWN           ( sCurrState.conButton[2] )
   #define BUTTON_REDCLIMBERDOWN_PREV      ( sPrevState.conButton[2] )
      // Console button  4 (the centermost pushbutton switch on the console)
      // runs the conveyor backward.
#define BUTTON_CONVEYORBACKWARD        ( sCurrState.conButton[8] )
#define BUTTON_CONVEYORBACKWARD_PREV   ( sPrevState.conButton[8] )
      // No button currently set for camera switching
      // 13 does not exist
      // defines kept for future debugging use
#define BUTTON_SWITCHCAMERA           ( sCurrState.conButton[13] )
#define BUTTON_SWITCHCAMERA_PREV      ( sPrevState.conButton[13] )
      // Console button  6 (leftmost bottom pushbutton switch on the console)
      // flips up the color wheel
#define BUTTON_UPPYDOWNEY             ( sCurrState.conButton[6] )
#define BUTTON_UPPYDOWNEY_PREV        ( sPrevState.conButton[6] )
      // Console button  8 (rightmost bottom pushbutton switch on the console)
      // turns on the intake motor
#define BUTTON_RUNINTAKE              ( sCurrState.conButton[7] )
#define BUTTON_RUNINTAKE_PREV         ( sPrevState.conButton[7] )
         // Console button 12 is the leftmost missile switch.
#define BUTTON_SWITCH1 ( sCurrState.conButton[12] )
#define BUTTON_SWITCH2 ( sCurrState.conButton[9]  )
#define BUTTON_SWITCH3 ( sCurrState.conButton[10] )
#define BUTTON_SWITCH4 ( sCurrState.conButton[11] )

              /* limelight variables: x: offset from vertical centerline,   */
              /*                      y: offset from horizontal centerline, */
              /*                      a: area of target, % (0-100),         */
              /*                      v: whether the data is valid,         */
              /*                      s: skew or rotation, deg (-90-0).     */
double limex, limey, limea, limev, limes;


   static void VisionThread()
   {
#define DETECT_APRILTAGS 1
// #undef DETECT_APRILTAGS 

#ifdef DETECT_APRILTAGS
      frc::AprilTagDetector detector;
      // Looks for tag16h5, don't correct any error bits
      detector.AddFamily("tag16h5", 0);

      // Parameters are for a Microsoft Lifecam HD-3000
      // We still need to measure these parameters for the new global shutter
      // camera, which should be much better at seeing/detecting apriltags.
      frc::AprilTagPoseEstimator::Config poseEstConfig = {
          .tagSize = units::length::inch_t(6.0),
          .fx = 699.3778103158814,
          .fy = 677.7161226393544,
          .cx = 345.6059345433618,
          .cy = 207.12741326228522};
      frc::AprilTagPoseEstimator estimator =
                                    frc::AprilTagPoseEstimator(poseEstConfig);
#endif  // DETECT_APRILTAGS

      // Get the USB camera from CameraServer
      cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
      // Set the resolution
      camera.SetResolution(640, 480);
      camera.SetExposureManual(25);

      // Get a CvSink. This will capture Mats from the Camera
      cs::CvSink cvSink = frc::CameraServer::GetVideo();
      // Setup a CvSource. This will send images back to the Dashboard
      cs::CvSource outputStream =
          frc::CameraServer::PutVideo("DriveCam", 640, 480);

      // Mats are very memory expensive. Lets reuse this Mat.
      cv::Mat mat;
#ifdef DETECT_APRILTAGS
      cv::Mat grayMat;

      // Instantiate once
      std::vector<int> tags;
      cv::Scalar outlineColor = cv::Scalar(0, 255, 0);
      cv::Scalar crossColor = cv::Scalar(0, 0, 255);
#endif  // DETECT_APRILTAGS

      while (true)
      {
         // Tell the CvSink to grab a frame from the camera and put it
         // in the source mat.  If there is an error notify the output.
         if (cvSink.GrabFrame(mat) == 0)
         {
            // Send the output the error.
            outputStream.NotifyError(cvSink.GetError());
            usleep( 2000 );                             // wait 2 milliseconds
                                     // skip the rest of the current iteration
            continue;
         }

#ifdef DETECT_APRILTAGS
         cv::cvtColor(mat, grayMat, cv::COLOR_BGR2GRAY);

         cv::Size g_size = grayMat.size();
         frc::AprilTagDetector::Results detections =
             detector.Detect(g_size.width, g_size.height, grayMat.data);

         tags.clear();                           // have not seen any tags yet

         for (const frc::AprilTagDetection *detection : detections)
         {
            tags.push_back(detection->GetId());    // remember we saw this tag

            for (int i = 0; i <= 3; i++)          // draw lines around the tag
            {
               int j = (i + 1) % 4;
               const frc::AprilTagDetection::Point pti =
                                                      detection->GetCorner(i);
               const frc::AprilTagDetection::Point ptj =
                                                      detection->GetCorner(j);
               line( mat, cv::Point(pti.x, pti.y), cv::Point(ptj.x, ptj.y),
                     outlineColor, 2 );
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

                                                    // put pose into dashboard
            std::stringstream dashboardString;
            dashboardString << "Translation: "
                            << units::length::to_string(pose.X()) << ", "
                            << units::length::to_string(pose.Y()) << ", "
                            << units::length::to_string(pose.Z());
            frc::Rotation3d rotation = pose.Rotation();
            dashboardString << "; Rotation angles: "
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

                                   // Convert units::length::meter_t to double
               xid3 = units::length::meter_t(pose.X()).to<double>();
               zid3 = units::length::meter_t(pose.Z()).to<double>();

                        // Convert units::angle::radian_t to degrees in double
               rotationid3 = units::angle::degree_t(rotation.Y()).to<double>();

               zid3 = zid3 - 0.75;         // Offset (distance from tracker m)

               xid3 = xid3 / 1;                       // Divide to make slower
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

               std::cout << "Sideways: " << xid3 << " Depth: " << zid3
                         << " Rotation: " << rotationid3 << std::endl;
            }
         }

         std::stringstream tags_s;          // put list of tags onto dashboard
         if (tags.size() > 0)
         {
            if (tags.size() > 1)
            {
               std::copy( tags.begin(), tags.end() - 1,
                          std::ostream_iterator<int>(tags_s, ",") );
            }
            tags_s << tags.back();
         }
         frc::SmartDashboard::PutString( "tags", tags_s.str() );
#endif  // DETECT_APRILTAGS

                              // Give the output stream a new image to display
         outputStream.PutFrame(mat);

         usleep( 2000 );                                // wait 2 milliseconds
      }   // while ( true )
   }  // VisionThread()

      /*---------------------------------------------------------------------*/
      /* GetAllVariables()                                                   */
      /* Retrieves all variable values from sensors, encoders,               */
      /* the limelight, etc.  It should be called at the beginning of        */
      /* every 20-millisecond tick.  Doing it this way, rather than          */
      /* having each function retrieve the values it needs when it needs     */
      /* them, should minimize CANbus traffic and keep the robot CPU fast.   */
      /*---------------------------------------------------------------------*/
   void GetAllVariables()  {

      static int iCallCount = 0;
      iCallCount++;
                                // use frc::Timer::GetFPGATimestamp() instead?
      // dTimeOfLastCall = frc::GetTime();
      //      cout << std::setw( 20 ) << std::setprecision( 16 ) <<
      //              dTimeOfLastCall << " ";

      sPrevState = sCurrState;                  // save all previous variables

      sCurrState.conX = m_Console.GetX();
      sCurrState.conY = m_Console.GetY();
      for ( int iLoopCount=1; iLoopCount<=12; iLoopCount++ ) {
         sCurrState.conButton[iLoopCount] = m_Console.GetRawButton(iLoopCount);
      }
      // sCurrState.yawPitchRoll[0] = (double)gyro.GetAngle();
      // sCurrState.yawPitchRoll[1] = (double)gyro.GetRate();
      // sCurrState.rateXYZ[2]      = (double)sCurrState.yawPitchRoll[1];

      limev = limenttable->GetNumber("tv",0.0);  // valid
      limex = limenttable->GetNumber("tx",0.0);  // x position
      limea = limenttable->GetNumber("ta",0.0);  // area
      limey = limenttable->GetNumber("ty",0.0);  // y position
      limes = limenttable->GetNumber("ts",0.0);  // skew

   }   // GetAllVariables()


   /*---------------------------------------------------------------------*/
   /* ArmMotorInitSpark()                                                 */
   /* Setup the initial configuration of a Neo motor, driven by a         */
   /* Spark Max controller.  These settings can be superseded after this  */
   /* function is called, for the needs of each specific SparkMax-driven  */
   /* motor.                                                              */
   /*---------------------------------------------------------------------*/
   void ArmMotorInitSpark(rev::CANSparkMax &m_motor)
   {
      // Set argument to true to also burn defaults into SparkMax flash.
      m_motor.RestoreFactoryDefaults(false);

      m_motor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward,
                              false);
      m_motor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse,
                              false);
      m_motor.SetInverted(false); // set forward direction of motor.

      /* Set limits to how much current will be sent through the motor */
#ifdef SAFETY_LIMITS
                       //  2 Amps below 5000 RPM, above 5000 RPM it ramps from
                       //  2 Amps down to  1 Amp  at 5700 RPM
      m_motor.SetSmartCurrentLimit( 2, 1, 5000);
#else
                       // 20 Amps below 5000 RPM, above 5000 RPM it ramps from
                       // 20 Amps down to 10 Amps at 5700 RPM
      m_motor.SetSmartCurrentLimit(20, 10, 5000);
#endif
      m_motor.GetForwardLimitSwitch(
                                rev::SparkMaxLimitSwitch::Type::kNormallyOpen)
                                     .EnableLimitSwitch(false);

   // Config 100% motor output to 12.0V
      m_motor.EnableVoltageCompensation(12.0);

      // Set ramp rate (how fast motor accelerates or decelerates).
      // We may have to try different RampRates here to
      // eliminate drivetrain chattering.
      m_motor.SetClosedLoopRampRate(0.1);
      m_motor.SetOpenLoopRampRate(0.1);

      // m_motor.SetIdleMode( rev::CANSparkMax::IdleMode::kCoast );
      m_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

   } // ArmMotorInitSpark()

   /*---------------------------------------------------------------------*/
   /* ArmMotorInitTalon()                                                 */
   /* Setup the initial configuration of a brushed motor, driven by a     */
   /* Talon SRX controller.  These settings can be superseded after this  */
   /* function is called, for the needs of each specific Talon-driven     */
   /* motor.                                                              */
   /*---------------------------------------------------------------------*/
   void ArmMotorInitTalon( WPI_TalonSRX &m_motor )
   {
      m_motor.ConfigFactoryDefault( 10 );
      m_motor.SetSensorPhase(true);   // invert encoder value positive/negative
      m_motor.SetInverted(false);     // invert direction of motor itself.

                /* Configure Sensor Source for Primary PID */
          /* Config to stop motor immediately when limit switch is closed. */
                                                   // if encoder is connected
      if ( OK == m_motor.ConfigSelectedFeedbackSensor(
                     FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
//       m_motor.ConfigForwardLimitSwitchSource(
//                   LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
//                   LimitSwitchNormal::LimitSwitchNormal_NormallyOpen );
//       m_motor.ConfigReverseLimitSwitchSource(
//                   LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
//                   LimitSwitchNormal::LimitSwitchNormal_NormallyOpen );
          m_motor.OverrideLimitSwitchesEnable(true);
      }

         /*
          * Configure Talon SRX Output and Sensor direction.
          * Invert Motor to have green LEDs when driving Talon Forward
          * ( Requesting Positive Output ),
          * Phase sensor to have positive increment when driving Talon Forward
          * (Green LED)
          */
      m_motor.SetSensorPhase(true);   // invert encoder value positive/negative
      m_motor.SetInverted(false);     // invert direction of motor itself.

                        /* Set relevant frame periods to be at least as fast */
                        /* as the periodic rate.                             */
      m_motor.SetStatusFramePeriod(
                         StatusFrameEnhanced::Status_13_Base_PIDF0,  10, 10 );
      m_motor.SetStatusFramePeriod(
                         StatusFrameEnhanced::Status_10_MotionMagic, 10, 10 );

                                         /* Set the peak and nominal outputs */
      m_motor.ConfigNominalOutputForward( 0, 10 );
      m_motor.ConfigNominalOutputReverse( 0, 10 );
      m_motor.ConfigPeakOutputForward(    1, 10 );
      m_motor.ConfigPeakOutputReverse(   -1, 10 );

            /* Set limits to how much current will be sent through the motor */
      m_motor.ConfigPeakCurrentDuration(1);  // 1000 milliseconds (for 60 Amps)
#ifdef SAFETY_LIMITS
      m_motor.ConfigPeakCurrentLimit(10);       // limit motor power severely
      m_motor.ConfigContinuousCurrentLimit(10); // to 10 Amps
#else
      m_motor.ConfigPeakCurrentLimit(60);        // 60 works here for miniCIMs,
                                                 // or maybe 40 Amps is enough,
                                                 // but we reduce to 10, 1, 10
      m_motor.ConfigContinuousCurrentLimit(60);  // for safety while debugging
#endif
      m_motor.EnableCurrentLimit(true);

                                          // Config 100% motor output to 12.0V
      m_motor.ConfigVoltageCompSaturation( 12.0 );
      m_motor.EnableVoltageCompensation( false );

                 /* Set Closed Loop PIDF gains in slot0 - see documentation */
                                                    // if encoder is connected
      if ( OK == m_motor.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motor.SelectProfileSlot( 0, 0 );
         m_motor.Config_kF( 0, 0.15,   10 );
         m_motor.Config_kP( 0, 0.2,    10 );
         m_motor.Config_kI( 0, 0.0002, 10 );
         m_motor.Config_kD( 0, 10.0,   10 );
      } else {
         m_motor.SelectProfileSlot( 0, 0 );
         m_motor.Config_kF( 0, 0.15, 10 );
         m_motor.Config_kP( 0, 0.0, 10 );
         m_motor.Config_kI( 0, 0.0, 10 );
         m_motor.Config_kD( 0, 0.0, 10 );
      }

                /* Set acceleration and cruise velocity - see documentation */
      m_motor.ConfigMotionCruiseVelocity( 1500, 10 );
      m_motor.ConfigMotionAcceleration(   1500, 10 );

               /* Set ramp rate (how fast motor accelerates or decelerates) */
      m_motor.ConfigClosedloopRamp(0.1);
      m_motor.ConfigOpenloopRamp(  0.1);

      m_motor.SetNeutralMode( NeutralMode::Brake );

   }      // MotorInitTalon()


   bool DriveToPose( frc::Pose2d DestinationPose )
   {
      static bool bReturnValue = true;
      static int iCallCount = 0;

      frc::Transform2d transform = DestinationPose - 
                               m_swerve.m_poseEstimator.GetEstimatedPosition();
      const auto xSpeed = m_xspeedLimiter.Calculate(
                        transform.X().value() / 1.0 ) * Drivetrain::kMaxSpeed;
      const auto ySpeed = m_yspeedLimiter.Calculate(
                        transform.Y().value() / 1.0 ) * Drivetrain::kMaxSpeed;
      const auto rot =
            -m_rotLimiter.Calculate( transform.Rotation().Degrees().value() /
                                       180.00 ) * Drivetrain::kMaxAngularSpeed;
      if ( 4 == iCallCount%50 ) {
         std::cout << "xSpeed: " << xSpeed.value() << std::endl;
      }

      m_swerve.Drive(xSpeed, ySpeed, rot, true);

                    // If any (X/Y/Rotation) of the criteria for being
                    // at the desired pose (within 10 cm X and Y, and
                    // within 10 degrees yaw) are still not met...
      if ( ( (units::length::meter_t)0.10 < transform.X() ) ||
	   ( transform.X() < (units::length::meter_t) -0.10 ) ||
           ( transform.Y() < (units::length::meter_t) -0.10 ) ||
	   ( (units::degree_t)-10.0 > transform.Rotation().Degrees() ) ||
           ( (units::length::meter_t)0.10 < transform.Y() ) ||
           ( (units::degree_t)10.0 < transform.Rotation().Degrees() ) ) {
         bReturnValue = false;  // we are not yet at the desired pose
      } else {
         bReturnValue = true;   // we are at the desired pose
      }
      iCallCount++;
      return bReturnValue;
   }


   bool DriveToBalance( )
   {
      static bool bReturnValue = true;
      static int iCallCount = 0;
      static int iFlatCount = 0;  // how many times has this function been
                                  // called since the robot became flat?

               // DriveUphill returns true when the robot is flat (horizontal)
               // or is going to be flat (the charging station is pitched up,
               // but is pitching down).
      bReturnValue = m_swerve.DriveUphill(
                                  (units::velocity::meters_per_second_t)0.0 );

      if ( bReturnValue ) {
         iFlatCount++;
      } else {
         iFlatCount = 0;
      }

      iCallCount++;
                                 // Only return true if the robot is flat, and
                                 // *has been flat* for at least 1.5 seconds.
      return bReturnValue && ( 75 < iFlatCount );
   }


   void DriveWithJoystick(bool fieldRelative)

   {
      static int iCallCount = 0;

      iCallCount++;
      if (0 == iCallCount % 50)
      {
         // std::cout << "Joy X/Y/RotX " << m_DriveController.GetLeftX()  <<
         //                          "/" << m_DriveController.GetLeftY()  <<
         //                          "/" << m_DriveController.GetRightX() <<
         //                                                         std::endl;
      }

      // if not "A" button on DriveController
      if (!m_DriveController.GetAButton())
      {
         // Get the x speed. We are inverting this because Xbox controllers
         // return negative values when we push forward.
         const auto xSpeed = -m_xspeedLimiter.Calculate(
                      frc::ApplyDeadband(m_DriveController.GetLeftY(), 0.10)) *
                             Drivetrain::kMaxSpeed * speedfactor;

         // Get the y speed or sideways/strafe speed. We are inverting this
         // because we want a positive value when we pull to the left.
         // Xbox controllers return positive values when you pull to the right
         // by default.
         const auto ySpeed = -m_yspeedLimiter.Calculate(
                      frc::ApplyDeadband(m_DriveController.GetLeftX(), 0.10)) *
                             Drivetrain::kMaxSpeed * speedfactor;

         // Get the rate of angular rotation. We are inverting this because 
         // we want a positive value when we pull to the left (remember:
         // CCW is positive in mathematics). Xbox controllers return positive
         // values when you pull to the right by default.
         const auto rot = -m_rotLimiter.Calculate(
                    frc::ApplyDeadband(m_DriveController.GetRightX(), 0.10)) *
                          Drivetrain::kMaxAngularSpeed * speedfactor;

         m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);
      } else {   // else "A" button is pressed on DriveController
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
      }   // "A" button is pressed on DriveController
   }

         /*---------------------------------------------------------------------*/
      /* executeManeuver()                                                   */
      /* This function is called with a maneuver struct, to perform a        */
      /* specified maneuver and return the completion status.                */
      /* It returns false if the maneuver is not yet completed, or           */
      /* true if the maneuver has been completed.                            */
      /*---------------------------------------------------------------------*/
   bool executeManeuver( struct maneuver mSeq ) {
      bool       bRetVal = false;
      static struct maneuver mSeqPrev = { 0,  M_STOP,
                                   { (units::meter_t)0.0,
                                     (units::meter_t)0.0,
                                     (units::degree_t)0.0 }, 0.0, false };
      // static int icmdSeqManeuverCallCount = 0;
      // static int iLimeLockCount = 100;
      static double dWaitCount =  50.0; // ticks to wait (50 ticks = 1 second)

      //              // print debugging info (this code should be removed later)
      // if ( ( mSeqPrev.index != mSeq.index ) ||
      //      ( mSeqPrev.type  != mSeq.type  )    ) {
      //       cout << "executeManeuver: Changed Maneuver, index: ";
      //       cout << mSeqPrev.index << " > " << mSeq.index << " ." << endl;
      //       cout << "                                    type: ";
      //       cout << mSeqPrev.type  << " > " << mSeq.type  << " ." << endl;
      //       cout << "                                distance: ";
      //       cout << mSeqPrev.distance << " > " << mSeq.distance << " ." << endl;
      //       cout << "                                     yaw: ";
      //       cout << mSeqPrev.yaw   << " > " << mSeq.yaw << " ." << endl;
      // }

      switch ( mSeq.type )
      {
      case M_TERMINATE_SEQ:
                                          // Make sure drive motors are stopped
         m_swerve.Drive( (units::velocity::meters_per_second_t)0.0,
                         (units::velocity::meters_per_second_t)0.0,
                         (units::angular_velocity::radians_per_second_t)0.0,
                         false ); // Robot-centric drive, not field-oriented
         if ( mSeqPrev.type != mSeq.type ) {
            cout << "EM: M_TERMINATE_SEQ: no further movement will happen!";
         }
         bRetVal = false;                 // and stay in this maneuver forever.
         break;

      case M_STOP:
                                          // make sure drive motors are stopped
         m_swerve.Drive( (units::velocity::meters_per_second_t)0.0,
                         (units::velocity::meters_per_second_t)0.0,
                         (units::angular_velocity::radians_per_second_t)0.0,
                         false ); // Robot-centric drive, not field-oriented
         bRetVal = true;                  // and exit this maneuver immediately
         break;

      case M_GO_TO_POSE:
         // Drive straight to the specified pose.
         // The DriveToPose() function returns true when it is near the pose.
         bRetVal = DriveToPose( mSeq.DestinationPose );

                                           // If we have driven far enough...
         if ( bRetVal ) {
            frc::Pose2d currPose =
                               m_swerve.m_poseEstimator.GetEstimatedPosition();
            cout << "EM: DriveToPose() completed, final X/Y/Rot: ";
            cout << currPose.X().value() << "/" << currPose.Y().value() <<
                    "/" << currPose.Rotation().Degrees().value() << endl;
         }
         break;

      case M_WAIT:
         if ( mSeqPrev.type != mSeq.type ) {          // first call to M_WAIT?
                                         // yes; store number of ticks to wait
            dWaitCount = (int)mSeq.dArg;
         } else {
            dWaitCount -= 1.0;
         }
                  // If we have waited long enough,
         if ( dWaitCount <= 0.0 ) {
            cout << "EM: M_WAIT completed" << endl;
            bRetVal = true;          // done waiting; change to next maneuver.
         } else {
            bRetVal = false;         // stay in this maneuver; keep waiting.
         }
         break;

      case M_BALANCE:
         // Drive straight to the specified pose.
         // The DriveToPose() function returns true when it is near the pose.
         bRetVal = DriveToBalance();

                          // If we are balanced (flat on the charging station)
         if ( bRetVal ) {
            frc::Pose2d currPose =
                               m_swerve.m_poseEstimator.GetEstimatedPosition();
            cout << "EM: DriveToBalance() completed, final X/Y/Rot: ";
            cout << currPose.X().value() << "/" << currPose.Y().value() <<
                    "/" << currPose.Rotation().Degrees().value() << endl;
         }
         break;

      default:
         cout << "EM: ERROR: Unknown maneuver type: ";
         cout << mSeq.type << "." << endl;
         break;
      }

      mSeqPrev = mSeq;   // Save a copy of this maneuver, to compare with
                         // the next maneuver we get.

      return bRetVal;
   }   // executeManeuver()
       //
       //
       //
   /*---------------------------------------------------------------------*/
      /* executeManeuverSeq()                                                */
      /* This function is called with an index into the maneuver array,      */
      /* to perform a sequence of maneuvers starting at that index.          */
      /* Each maneuver will be performed to completion, then the next        */
           /* executeManeuverSeq()                                                */
      /* This function is called with an index into the maneuver array,      */
      /* to perform a sequence of maneuvers starting at that index.          */
      /* Each maneuver will be performed to completion, then the next        */
      /* maneuver in the array will be started.                              */
      /* This function returns the index of the maneuver which should be     */
      /* executed on the next call (20 milliseconds later); until a          */
      /* maneuver is completed this will be same index it was called with.   */
      /*---------------------------------------------------------------------*/
   int executeManeuverSequence( int maneuverIndex ) {
      struct maneuver mSeqNext;
      // static int icmdSeqManeuverCallCount = 0;

                      // execute the current maneuver, and if it is finished...
      if ( executeManeuver( mSeq[ maneuverIndex ] ) ) {
         maneuverIndex++;                                // go to next maneuver
         mSeqNext = mSeq[maneuverIndex];
                                    // if next maneuver is a drive to distance,
         // if ( M_DRIVE_STRAIGHT == mSeqNext.type ) {
                                    // then initialize the starting point
         //   DriveToDistance( mSeqNext.yaw,
                           //  mSeqNext.distance,
                          //  mSeqNext.bDivertToCargo,
                           //  true );
            // } else if ( M_ROTATE == mSeqNext.type ) {
                      // Else if next maneuver is a rotate, then set the
                      // initial (starting) yaw angle.
                      // NO: call TurnToHeading(..., true) just once in
                      // AutonomousInit(), and nowhere else, so all yaw values
                      // are always based on the initial yaw of the robot when
                      // AutonomousInit() was called.
                      // This allows all yaw values in the maneuver struct
                      // to be absolute, and relative to the initial yaw angle
                      // of the robot at AutonomousInit() time.
            //    TurnToHeading( mSeqNext.yaw, true ) ) {
        // }
      }
                              // return either the current maneuver index, or
                              // if that just finished, the next maneuver index
      return maneuverIndex;
   }
                        

      /*---------------------------------------------------------------------*/
      /* RobotInit()                                                         */
      /* This function is called once when the robot is powered up.          */
      /* It performs preliminary initialization of all hardware that will    */
      /* be used in Autonomous or Teleop modes.                              */
      /*---------------------------------------------------------------------*/
   void RobotInit() override
   {
      static int iRobotInitCallCount = 0;

      iRobotInitCallCount++;
      
      if ( 1 == iRobotInitCallCount++ ) {
         std::thread visionThread( VisionThread );
         visionThread.detach();
      }
   }  // RobotInit()

      /*---------------------------------------------------------------------*/
      /* RobotPeriodic()                                                     */
      /* This function is called every 20 milliseconds, regardless of what   */
      /* mode the robot is in (Autonomous, Teleop, or Test mode).            */
      /* If the robot is in one of those modes, this function is called      */
      /* immediately *after* the call to AutonomousPeriodic(),               */
      /* TeleopPeriodic(), or TestPeriodic().                                */
      /* If not in one of those 3 modes, the Roborio cannot drive any        */
      /* motors, but it can still check the joystick, joystick/console       */
      /* buttons, and sensors.                                               */
      /*---------------------------------------------------------------------*/
   void RobotPeriodic() override
   {
      if (m_DriveController.GetBackButton())
      {
         m_swerve.Reset();
      }
      if (m_DriveController.GetBButton())
      {
         speedfactor = 0.2;
      } else {
         speedfactor = 1.0;
      }
   }  // RobotPeriodic()

      /*---------------------------------------------------------------------*/
      /* DisabledInit()                                                      */
      /* This function is called once when the robot is disabled.            */
      /*---------------------------------------------------------------------*/
   void DisabledInit() override {
   }  // DisabledInit()

      /*---------------------------------------------------------------------*/
      /* DisabledPeriodic()                                                  */
      /* This function is called every 20 milliseconds, as long as the robot */
      /* is in Disabled mode.                                                */
      /*---------------------------------------------------------------------*/
   void DisabledPeriodic() override {
   }  // DisabledPeriodic()

      /*---------------------------------------------------------------------*/
      /* TestInit()                                                          */
      /* This function is called once when the robot enters Test mode.       */
      /*---------------------------------------------------------------------*/
   void TestInit() override {
   }  // TestInit()

      /*---------------------------------------------------------------------*/
      /* TestPeriodic()                                                      */
      /* This function is called every 20 milliseconds, as long as the robot */
      /* is in Test mode.                                                    */
      /* In this mode the Roborio cannot drive any motors, but it can read   */
      /* the joystick, joystick/console buttons, and sensors.                */
      /*---------------------------------------------------------------------*/
   void TestPeriodic() override {
   }  // TestPeriodic()


      /*---------------------------------------------------------------------*/
      /* AutonomousInit()                                                    */
      /* This function is called once when the robot enters Autonomous mode. */
      /*---------------------------------------------------------------------*/
   void AutonomousInit() override {
      RobotInit();
   // m_compressor.EnableDigital();
      m_compressor.Disable();
      iCallCount = 0;
      m_swerve.Reset();

      GetAllVariables();
      sCurrState.teleop = false;
      sCurrState.initialYaw = sCurrState.yawPitchRoll[0];

                                   // mSeqIndex can be set to different values,
                             // based on the console switches.
      if ( BUTTON_SWITCH1 ) {
         mSeqIndex =  0;         // simple auto (forward, right, back, balance) 
      } else if ( BUTTON_SWITCH2 ) {
         mSeqIndex = 10;               // simple auto (forward, back, balance)
      } else if ( BUTTON_SWITCH3 ) {
         mSeqIndex = 20;         // simple auto (forward, left, back, balance)
      } else if ( BUTTON_SWITCH4 ) {
         mSeqIndex = 30;	 // test auto (various sequences) 
      } else {
         // mSeqIndex = 10;             // forward, back, balance
         mSeqIndex =  0;             // no switch flipped, do nothing
                                     // (points to M_TERMINATE_SEQ, for safety)
      }
   }   // AutonomousInit()

   void AutonomousPeriodic() override
   {
      GetAllVariables();

#ifndef JAG_NOTDEFINED
                            // Perform a sequence of maneuvers, transitioning
                            // to next maneuver in the sequence when necessary.
      mSeqIndex = executeManeuverSequence( mSeqIndex );
#else
      // DriveWithJoystick(false);
      DriveToPose( DestinationOne );
#endif
      m_swerve.UpdateOdometry();


      return;
   }   // AutonomousPeriodic()


   void TeleopInit() override
   {
      ArmMotorInitTalon( m_ExtenderMotor );
      ArmMotorInitTalon( m_WristMotor );
      m_ExtenderMotor.ConfigPeakOutputForward(  1.0, 10 );
      m_ExtenderMotor.ConfigPeakOutputReverse( -1.0, 10 );
      m_WristMotor.ConfigPeakOutputForward(  1.0, 10 );
      m_WristMotor.ConfigPeakOutputReverse( -1.0, 10 );
//      m_compressor.EnableDigital();
      m_compressor.Disable();
   }   // TeleopInit()


   // change to true for field relative
   //          false for robot relative
   void TeleopPeriodic() override
   {
      static int  iCallCount = 0;
      static bool bGrabberPortState = false;    // Is portside  grabber closed?
      static bool bGrabberStbdState = false;    // Is starboard grabber closed?

      static bool bBButton = false;
      static bool bBButton_prev = false;

             bool bXButton = m_OperatorController.GetXButton();
      static bool bXButton_prev = false;

      static double dArmDirection = 1.0;

      m_swerve.UpdateOdometry();

      if ( !bXButton_prev && bXButton ) {
         if ( dArmDirection < 0.0 ) {
            dArmDirection =  1.0;
         } else {
            dArmDirection = -1.0;
         }
      }
      bXButton_prev = bXButton;

#ifdef JAG_NOTDEFINED
      if ( 0 == iCallCount%500 ) {
         std::cout << "Compressor Enabled?: " << m_compressor.IsEnabled() <<
               "  PressureSwitch: " << m_compressor.GetPressureSwitchValue() <<
               // units::pressure::pounds_per_square_inch_t
               "  Press: " << m_compressor.GetPressure().value() << " PSI " <<
               // units::current::ampere_t
               "  Current: " << m_compressor.GetCurrent().value() << " A" <<
               std::endl;
      }
#endif
      iCallCount++;


      DriveWithJoystick(true);
                                    // Run the shoulder motor to rotate the arm
      m_ShoulderMotor.SetVoltage(
        units::volt_t{
                     // Be very careful with this motor; it is geared down so
                     // low that it could easily break things if left here
                     // at a high amp limit (high torque limit)
         frc::ApplyDeadband( dArmDirection * m_OperatorController.GetRightY(),
                             0.10) } *
#ifdef SAFETY_LIMITS
                                        2.0 );
#else
                                       12.0 );
#endif
                                    // Run the extender motor to extend the arm
      if ( 2 == iCallCount%50 ) {
         std::cout << "LeftTrigger: " <<
                       m_OperatorController.GetLeftTriggerAxis() << std::endl;
         std::cout << "RightTrigger: " <<
                       m_OperatorController.GetRightTriggerAxis() << std::endl;
      }
      if ( 0.10 < m_OperatorController.GetLeftTriggerAxis() ) {
                     // Be very careful with this motor; it is geared down so
                     // low that it could easily break things if left here
                     // at a high amp limit (high torque limit)
         m_ExtenderMotor.SetVoltage(
            units::volt_t{
        -frc::ApplyDeadband(m_OperatorController.GetLeftTriggerAxis(),
                            0.10) } *
#ifdef SAFETY_LIMITS
                                       4.0 );
#else
                                      12.0 );
#endif
      } else if ( 0.10 < m_OperatorController.GetRightTriggerAxis() ) {
                     // Be very careful with this motor; it is geared down so
                     // low that it could easily break things if left here
                     // at a high amp limit (high torque limit)
         m_ExtenderMotor.SetVoltage(
            units::volt_t{
         frc::ApplyDeadband(m_OperatorController.GetRightTriggerAxis(),
                            0.10) } *
#ifdef SAFETY_LIMITS
                                       4.0 );
#else
                                      12.0 );
#endif
      } else {
         m_ExtenderMotor.SetVoltage( units::volt_t{ 0.0 } );
      }
                                    // Run the wrist motor to rotate the arm
      m_WristMotor.SetVoltage(
        units::volt_t{
                     // Be very careful with this motor; it is geared down so
                     // low that it could easily break things if left here
                     // at a high amp limit (high torque limit)
         frc::ApplyDeadband( dArmDirection * m_OperatorController.GetLeftY(),
                             0.10) } *
#ifdef SAFETY_LIMITS
                                       6.0 );
#else
                                       12.0 );
#endif

                                                  // open or close the grabber
      bBButton = m_OperatorController.GetBButton();
      if ( bBButton && !bBButton_prev ) {
         if ( bGrabberPortState || bGrabberStbdState ){
                                                        // open the grabber
            m_grabberPortSolenoid.Set( frc::DoubleSolenoid::Value::kReverse);
            m_grabberStbdSolenoid.Set( frc::DoubleSolenoid::Value::kReverse);
            bGrabberPortState = bGrabberStbdState = false; 

         } else {
                                                        // close the grabber
            m_grabberPortSolenoid.Set( frc::DoubleSolenoid::Value::kForward);
            m_grabberStbdSolenoid.Set( frc::DoubleSolenoid::Value::kForward);
            bGrabberPortState = bGrabberStbdState = true;                       

         }
      }
      bBButton_prev = bBButton;

   }   // TeleopPeriodic()

};

#ifndef RUNNING_FRC_TESTS
int main()
{
   return frc::StartRobot<Robot>();
}
#endif
