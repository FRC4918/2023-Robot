// Copyright (c) 2023 FRC Team 4918.
// Some software is also Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// #define SAFETY_LIMITS 1
#undef SAFETY_LIMITS

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
double dSpeedFactor;
int pipelineindex = 0;
bool ldriver = 0;

frc::XboxController m_DriveController{0};
frc::XboxController m_OperatorController{1};
frc::Joystick       m_Console{3};                   // the number 3 USB device

const int ShoulderMotorDeviceID =  16;
rev::CANSparkMax m_ShoulderMotor{ ShoulderMotorDeviceID,
                                  rev::CANSparkMax::MotorType::kBrushless};
rev::SparkMaxRelativeEncoder m_ShoulderEncoder = m_ShoulderMotor.GetEncoder();

                                                            // CTRE compressor
frc::Compressor m_compressor{ 0, frc::PneumaticsModuleType::CTREPCM };

frc::DoubleSolenoid m_grabberPortSolenoid{
                                  0, frc::PneumaticsModuleType::CTREPCM, 0, 2};
frc::DoubleSolenoid m_grabberStbdSolenoid{
                                  0, frc::PneumaticsModuleType::CTREPCM, 1, 3};
WPI_TalonSRX m_ExtenderMotor{  3 };   // motor for arm extender
WPI_TalonSRX m_WristMotor{    12 };   // motor for arm wrist

frc::DigitalInput wristForwardLimitDIO0{0};
frc::DigitalInput wristReverseLimitDIO1{1};
frc::DigitalInput extenderForwardReverseLimitDIO2{2};

   // The values below work well if the robot is powered up with the
   // shoulder positioned all the way forward (toward the "front" of the
   // robot, where the battery is).
const double kShoulderEncoderMin = -180.0;   // toward the back of the robot.
const double kShoulderEncoderMax = 0.0;         // Starting position
const double kShoulderPositionOffsetDeg = 140.0; // Start position (deg)
const double kWristEncoderMin = -2306;       // toward the front of the robot.
const double kWristEncoderMax = 0;           // Starting position

                                           // create a list of wrist positions
enum WRIST_POSITION {
   M_WRIST_FULLY_BACK    = 0,
   M_WRIST_MIDDLE        = 1,
   M_WRIST_FULLY_FORWARD = 2
};
                                   // wrist position (whether at either limit)
enum WRIST_POSITION WristPosition = M_WRIST_FULLY_BACK;

                                        // create a list of extender positions
enum EXTENDER_POSITION {
   M_EXTENDER_FULLY_RETRACTED = 0,
   M_EXTENDER_MIDDLE          = 1,
   M_EXTENDER_FULLY_EXTENDED  = 2,
   M_EXTENDER_DO_NOT_CARE     = 3
};

class Robot : public frc::TimedRobot
{

   int iCallCount = 0;

private:

   Drivetrain m_swerve;

             // Slew rate limiters to make joystick inputs more gentle;
             // 1/5 second from 0 to 1 (x and y speedLimiters were {3 / 1_s}).
// jag; 07apr2023  (separated teleop/auto drive slews, and
//                  reduced teleop from 1/5 to 1/10)
   frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{ 10 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{ 10 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_rotLimiter{ 10 / 1_s};
             // Separate the slew rates for driving during autonomous
   frc::SlewRateLimiter<units::scalar> m_xspeedLimiterA{ 5 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_yspeedLimiterA{ 5 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_rotLimiterA{ 5 / 1_s};

   frc::SlewRateLimiter<units::scalar> m_ShoulderLimiter{ 5 / 1_s};

   std::shared_ptr<nt::NetworkTable> limenttable =
               nt::NetworkTableInstance::GetDefault().GetTable( "limelight" );

   struct ArmPose {
                              // Shoulder position (degrees, 0 is straight up)
      double                 dShoulderPosition;
      enum EXTENDER_POSITION eExtenderPosition;
      double                 dWristPosition;
      bool                   bGrabberClosed;
   };
                                            // create a list of maneuver types
   enum MANEUVER_TYPE {
      M_TERMINATE_SEQ  = 0,
      M_STOP           = 1,
      M_GO_TO_POSE     = 2,  // drive straight to a specified pose
      M_ARM_TO_POS     = 3,  // drive straight to a specified pose
      M_WAIT           = 4,  // wait for a period of time (a number of ticks)
      M_BALANCE        = 5,  // balance on the charging station (teeter-totter)
      M_JUMP           = 6   // Goto a maneuver other than the next one
   };

   struct ArmPose M_ARMPOS_HOME   = { 140.0, M_EXTENDER_FULLY_RETRACTED,
                                      -90.0, true };
   
   struct ArmPose M_ARMPOS_HICONE = { -50.0, M_EXTENDER_FULLY_EXTENDED,
                                      -70.0, true };
  
   struct ArmPose M_ARMPOS_LOCONE = { -60.0, M_EXTENDER_FULLY_RETRACTED,
                                      -70.0, true };

   struct ArmPose M_ARMPOS_FRONTGRAB = { 120.0, M_EXTENDER_FULLY_EXTENDED,
                                         80.0, true };

   struct ArmPose M_ARMPOS_BACKGRAB  = { -140.0, M_EXTENDER_FULLY_EXTENDED,
                                         -90.0, true };

   struct ArmPose M_ARMPOS_HUMNPLYRSTATN = { 25.0, M_EXTENDER_FULLY_RETRACTED,
                                             86.0, true };

   
                    // create a struct which can contain a full maneuver
                    // (type, desired pose, including yaw (heading), etc.)
   struct maneuver {
      int                index;    // index of this element in an array of
                                   // maneuvers
      enum MANEUVER_TYPE type;     // type of maneuver (stop, turn, etc.)
      frc::Pose2d        DestinationPose;  // pose to drive toward
      struct ArmPose     sArmPose; // Arm position (shoulder, wrist, etc.)
      int                iArg;     // General-purpose integer argument
      double             dArg;     // General-purpose argument (seconds, etc.)
      bool               bArg;     // General-purpose boolean argument
   };

   int mSeqIndex = 0;

                                        // Create a sequence of full maneuvers
   struct maneuver mSeq[256] =
   {
     // Perform a maneuver until the specified distance and heading
     // has been achieved (the robot is at the desired pose).
     // Distances are relative to the end of the previous maneuver;
     // headings are absolute, from the initial yaw when AutonomousInit()
     // was called.
     //
   //                        destination
   //                        pose (meters              int   double  boolean
   // index command          and degrees)              arg1   arg1    arg2  
   // ----- ---------------- ------------------------  ----  ------  -------
   // Index 00 autonomous sequence index (left side non-balancing):
   // score, move forward out of community
   // This, like all auto sequences, starts out with the upper cube drop.
   {   0,  M_ARM_TO_POS,    { (units::foot_t)1.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
   // next line was FULLY_RETRACTED  (all below too)
                            { 120.0, M_EXTENDER_DO_NOT_CARE,
                                0.0, true },
                            50,   0.0,    false },
   {   1,  M_ARM_TO_POS,    { (units::foot_t)1.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
   // next line was M_EXTENDER_FULLY_EXTENDED,  (all below too)
                            { 120.0, M_EXTENDER_DO_NOT_CARE,
                                0.0, true },
                            50,   0.0,    false },
   {   2,  M_ARM_TO_POS,    { (units::foot_t)1.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { -50.0, M_EXTENDER_DO_NOT_CARE,
                              -80.0, true },
                            50,   0.0,    false },
   {   3,  M_ARM_TO_POS,    { (units::foot_t)4.0,
                              (units::foot_t)1.4,
                              (units::degree_t)0.0 },
                            { -55.0, M_EXTENDER_DO_NOT_CARE,
                                    -90.0, false },
                            50,   0.0,    false },
   {   4,  M_GO_TO_POSE,    { (units::foot_t)6.0,
                              (units::foot_t)1.4,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE,
                                   -90.0, false },
                            0,   0.0,    false },
   {   5,  M_GO_TO_POSE,    { (units::foot_t)14.0,   // 14.5, 
                              (units::foot_t)0.5,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_EXTENDED,
                                70.0, false },
                            0,   0.0,    false },
   {   6,  M_GO_TO_POSE,    { (units::foot_t)16.25,       // was 16.1,  // 15.2
                              (units::foot_t)0.5,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_EXTENDED,
                                70.0, false },
                            0,   0.0,    false },
   {   7,  M_ARM_TO_POS,    { (units::foot_t)16.25,       // was 16.1,  // 15.2
                              (units::foot_t)0.5,
                              (units::degree_t)0.0 },
                            { 112.2, M_EXTENDER_DO_NOT_CARE,  // 120, FULLY_,
                               129.6, false },                // 80, 
                            50,   0.0,    false },
   {   8,  M_WAIT,          { (units::foot_t)16.25,           // was 16.1,
                              (units::foot_t)0.5,
                              (units::degree_t)0.0 },
                            { 112.2, M_EXTENDER_DO_NOT_CARE,  // 120, FULLY_,
                               129.6, false },                // 80, 
                            50,   0.0,    false },
   {   9,  M_ARM_TO_POS,    { (units::foot_t)16.25,           // was 16.1,
                              (units::foot_t)0.5,
                              (units::degree_t)0.0 },
                            { 112.2, M_EXTENDER_DO_NOT_CARE,  // 120, FULLY_,
                               129.6, true },                 // 70, 
                            50,   0.0,    false },
   {  10,  M_ARM_TO_POS,    { (units::foot_t)16.25,           // was 16.1,
                              (units::foot_t)0.5,
                              (units::degree_t)0.0 },
                            {  90.0, M_EXTENDER_DO_NOT_CARE, // FULLY_EXTENDED,
                               70.0, true },
                            0,   0.0,    false },
   {  11,  M_GO_TO_POSE,    { (units::foot_t)0.0,
                              (units::foot_t)1.67,
                              (units::degree_t)0.0 },
                            { -60.0, M_EXTENDER_DO_NOT_CARE,
                              -70.0, true },
                            0,   0.0,    false },
   {  12,  M_ARM_TO_POS,    { (units::foot_t)0.0,
                              (units::foot_t)1.67,
                              (units::degree_t)0.0 },
                            { -60.0, M_EXTENDER_DO_NOT_CARE, // high cone score
                              -70.0, false },
                            0,   0.0,    false },
   {  13,  M_STOP,          { (units::foot_t)0.0,
                              (units::foot_t)1.67,
                              (units::degree_t)0.0 },
                            { 140.0, M_EXTENDER_FULLY_EXTENDED,
                                  90.0, false },
                            0,   0.0,    false },
   {  14,  M_TERMINATE_SEQ, { (units::foot_t)14.0,
                              (units::foot_t)2.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  15,  M_TERMINATE_SEQ, { (units::foot_t)14.0,
                              (units::foot_t)2.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  16,  M_TERMINATE_SEQ, { (units::foot_t)14.0,
                              (units::foot_t)2.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  17,  M_TERMINATE_SEQ, { (units::foot_t)14.0,
                              (units::foot_t)2.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  18,  M_TERMINATE_SEQ, { (units::foot_t)14.0,
                              (units::foot_t)2.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  19,  M_TERMINATE_SEQ, { (units::foot_t)14.0,
                              (units::foot_t)2.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },

   // Index 20 autonomous sequence index (center, non-balancing):
   // score, move forward (across charging station) out of community
   {  20,  M_ARM_TO_POS,    { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 120.0, M_EXTENDER_DO_NOT_CARE,
                                0.0, true },
                            50,   0.0,    false },
   {  21,  M_ARM_TO_POS,    { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 120.0, M_EXTENDER_DO_NOT_CARE, // FULLY_EXTENDED,
                                0.0, true },
                            50,   0.0,    false },
   {  22,  M_ARM_TO_POS,    { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { -50.0, M_EXTENDER_DO_NOT_CARE,
                              -80.0, true },
                            50,   0.0,    false },
   {  23,  M_ARM_TO_POS,    { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { -55.0, M_EXTENDER_DO_NOT_CARE,
                              -90.0, false },
                            50,   0.0,    false },
   {  24,  M_GO_TO_POSE,    { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                              -90.0, false },
                            0,   0.0,    false },
   {  25,  M_STOP,          { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                   0.0, false },
                            0,   0.0,    false },
   {  26,  M_TERMINATE_SEQ, { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  27,  M_TERMINATE_SEQ, { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  28,  M_TERMINATE_SEQ, { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  29,  M_TERMINATE_SEQ, { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  30,  M_TERMINATE_SEQ, { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  31,  M_TERMINATE_SEQ, { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  32,  M_TERMINATE_SEQ, { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  33,  M_TERMINATE_SEQ, { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  34,  M_TERMINATE_SEQ, { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  35,  M_TERMINATE_SEQ, { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  36,  M_TERMINATE_SEQ, { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  37,  M_TERMINATE_SEQ, { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  38,  M_TERMINATE_SEQ, { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  39,  M_TERMINATE_SEQ, { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },

   // Index 40 autonomous sequence index (right-side, non-balancing):
   // score, move forward out of community
   {  40,  M_ARM_TO_POS,    { (units::foot_t)14.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 120.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                                0.0, true },
                            50,   0.0,    false },
   {  41,  M_ARM_TO_POS,    { (units::foot_t)14.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 120.0, M_EXTENDER_DO_NOT_CARE, // FULLY_EXTENDED,
                                0.0, true },
                            50,   0.0,    false },
   {  42,  M_ARM_TO_POS,    { (units::foot_t)14.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { -50.0, M_EXTENDER_DO_NOT_CARE,
                              -80.0, true },
                            50,   0.0,    false },
   {  43,  M_ARM_TO_POS,    { (units::foot_t)14.0,
                              (units::foot_t)-14.0,
                              (units::degree_t)0.0 },
                            { -55.0, M_EXTENDER_DO_NOT_CARE,
                              -90.0, false },
                            50,   0.0,    false },
// jag; 07apr2023  (changed right side auto to be mirror image of left side)
   {  44,  M_GO_TO_POSE,    { (units::foot_t)6.0,
                              (units::foot_t)-1.4,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE,
                                   -90.0, false },
                            0,   0.0,    false },
   {  45,  M_GO_TO_POSE,    { (units::foot_t)14.0,   // 14.5, 
                              (units::foot_t)-0.5,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_EXTENDED,
                                70.0, false },
                            0,   0.0,    false },
   {  46,  M_GO_TO_POSE,    { (units::foot_t)16.25,       // was 16.1,  // 15.2
                              (units::foot_t)-0.5,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_EXTENDED,
                                70.0, false },
                            0,   0.0,    false },
   {  47,  M_ARM_TO_POS,    { (units::foot_t)16.25,       // was 16.1,  // 15.2
                              (units::foot_t)-0.5,
                              (units::degree_t)0.0 },
                            { 112.2, M_EXTENDER_DO_NOT_CARE,  // 120, FULLY_,
                               129.6, false },                // 80, 
                            50,   0.0,    false },
   {  48,  M_WAIT,          { (units::foot_t)16.25,           // was 16.1,
                              (units::foot_t)-0.5,
                              (units::degree_t)0.0 },
                            { 112.2, M_EXTENDER_DO_NOT_CARE,  // 120, FULLY_,
                               129.6, false },                // 80, 
                            50,   0.0,    false },
   {  49,  M_ARM_TO_POS,    { (units::foot_t)16.25,           // was 16.1,
                              (units::foot_t)-0.5,
                              (units::degree_t)0.0 },
                            { 112.2, M_EXTENDER_DO_NOT_CARE,  // 120, FULLY_,
                               129.6, true },                 // 70, 
                            50,   0.0,    false },
   {  50,  M_ARM_TO_POS,    { (units::foot_t)16.25,           // was 16.1,
                              (units::foot_t)-0.5,
                              (units::degree_t)0.0 },
                            {  90.0, M_EXTENDER_DO_NOT_CARE, // FULLY_EXTENDED,
                               70.0, true },
                            0,   0.0,    false },
   {  51,  M_GO_TO_POSE,    { (units::foot_t)0.0,
                              (units::foot_t)-1.67,
                              (units::degree_t)0.0 },
                            { -60.0, M_EXTENDER_DO_NOT_CARE,
                              -70.0, true },
                            0,   0.0,    false },
   {  52,  M_ARM_TO_POS,    { (units::foot_t)0.0,
                              (units::foot_t)-1.67,
                              (units::degree_t)0.0 },
                            { -60.0, M_EXTENDER_DO_NOT_CARE, // high cone score
                              -70.0, false },
                            0,   0.0,    false },
   {  53,  M_STOP,          { (units::foot_t)0.0,
                              (units::foot_t)-1.67,
                              (units::degree_t)0.0 },
                            { 140.0, M_EXTENDER_FULLY_EXTENDED,
                                  90.0, false },
                            0,   0.0,    false },
   {  54,  M_TERMINATE_SEQ, { (units::foot_t)14.0,
                              (units::foot_t)-2.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },

   {  55,  M_TERMINATE_SEQ, { (units::foot_t)14.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  56,  M_TERMINATE_SEQ, { (units::foot_t)14.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  57,  M_TERMINATE_SEQ, { (units::foot_t)14.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                              0.0, false },
                            0,   0.0,    false },
   {  58,  M_TERMINATE_SEQ, { (units::foot_t)14.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                              0.0, false },
                            0,   0.0,    false },
   {  59,  M_TERMINATE_SEQ, { (units::foot_t)14.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                              0.0, false },
                            0,   0.0,    false },

   // Index 60 autonomous sequence index (left-side, balancing):
   // score, move forward out of community, shift right, back, balance
   {  60,  M_ARM_TO_POS,    { (units::foot_t)14.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 120.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                                0.0, true },
                            50,   0.0,    false },
   {  61,  M_ARM_TO_POS,    { (units::foot_t)14.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 120.0, M_EXTENDER_DO_NOT_CARE, // FULLY_EXTENDED,
                                0.0, true },
                            50,   0.0,    false },
   {  62,  M_ARM_TO_POS,    { (units::foot_t)14.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { -50.0, M_EXTENDER_DO_NOT_CARE,
                              -80.0, true },
                            50,   0.0,    false },
   {  63,  M_ARM_TO_POS,    { (units::foot_t)14.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { -55.0, M_EXTENDER_DO_NOT_CARE,
                              -90.0, false },
                            50,   0.0,    false },
   {  64,  M_GO_TO_POSE,    { (units::foot_t)14.0,
                              (units::foot_t)2.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                              -90.0, false },
                            0,   0.0,    false },
   // The two setpoints below add a little jog to the left to avoid the
   // charging station.  This slows things down, so we use the method above.
   // {  6x,  M_GO_TO_POSE,    { (units::foot_t)3.5,
   //                            (units::foot_t)1.5,
   //                            (units::degree_t)0.0 },
   //                          { 100.0, M_EXTENDER_FULLY_RETRACTED,
   //                            -90.0, false },
   //                          0,   0.0,    false },
   // {  6x,  M_GO_TO_POSE,    { (units::foot_t)14.0,
   //                            (units::foot_t)1.5,
   //                            (units::degree_t)0.0 },
   //                          { 100.0, M_EXTENDER_FULLY_RETRACTED,
   //                            -90.0, false },
   //                          0,   0.0,    false },
   {  65,  M_GO_TO_POSE,    { (units::foot_t)14.0,
                              (units::foot_t)-4.0,   // should be -6.0
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                              -90.0, false },
                            0,   0.0,    false },
            // Center of charging station is actually 98 inches from grid base
            // so first field should probably be 7.0 or 6.0, rather than 9.0 .
   {  66,  M_GO_TO_POSE,    { (units::foot_t)7.0,    // was 9.0
                              (units::foot_t)-4.0,   // should be -6.0
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                              -90.0, false },
                            0,   0.0,    false },
   {  67,  M_BALANCE,       { (units::foot_t)7.0,    // was 9.0
                              (units::foot_t)-4.0,   // should be -6.0
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE,
                                0.0, false },
                            0,   0.0,    false },
   {  68,  M_STOP,          { (units::foot_t)7.0,    // was 9.0
                              (units::foot_t)-4.0,   // should be -6.0
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  69,  M_TERMINATE_SEQ, { (units::foot_t)7.0,    // was 9.0
                              (units::foot_t)-4.0,   // should be -6.0
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                              0.0, false },
                            0,   0.0,    false },
   {  70,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)-6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                              0.0, false },
                            0,   0.0,    false },
   {  71,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)-6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  72,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)-6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  73,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)-6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  74,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)-6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  75,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)-6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  76,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)-6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  77,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)-6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  78,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)-6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  79,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)-6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },

   // Index 60 autonomous sequence index (center, balancing):
   // score, move forward (across charging station) out of community,
   // wait for charging station to settle, back, balance
   {  80,  M_ARM_TO_POS,    { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 120.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                                0.0, true },
                            50,   0.0,    false },
   {  81,  M_ARM_TO_POS,    { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 120.0, M_EXTENDER_DO_NOT_CARE, // FULLY_EXTENDED,
                                0.0, true },
                            50,   0.0,    false },
   {  82,  M_ARM_TO_POS,    { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { -50.0, M_EXTENDER_DO_NOT_CARE,
                              -80.0, true },
                            50,   0.0,    false },
   {  83,  M_ARM_TO_POS,    { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { -55.0, M_EXTENDER_DO_NOT_CARE,
                              -90.0, false },
                            50,   0.0,    false },
   {  84,  M_GO_TO_POSE,    { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                              -90.0, false },
                            0,   0.0,    false },
   {  85,  M_STOP,          { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                              -90.0, false },
                            0,   0.0,    false },
   {  86,  M_WAIT,          { (units::foot_t)8.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                              -90.0, false },
                            50,   0.0,    false },
   {  87,  M_GO_TO_POSE,    { (units::foot_t)8.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                                 0.0, false },
                            50,   0.0,    false },
   {  88,  M_BALANCE,       { (units::foot_t)8.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE,
                                0.0, false },
                            0,   0.0,    false },
   {  89,  M_STOP,          { (units::foot_t)8.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  90,  M_TERMINATE_SEQ, { (units::foot_t)8.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  91,  M_TERMINATE_SEQ, { (units::foot_t)8.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  92,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  93,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  94,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  95,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  96,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  97,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  98,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   {  99,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },

   // Index 100 autonomous sequence index (right-side, balancing):
   // score, move forward out of community, shift left, back, balance
   { 100,  M_ARM_TO_POS,    { (units::foot_t)14.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 120.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                                0.0, true },
                            50,   0.0,    false },
   { 101,  M_ARM_TO_POS,    { (units::foot_t)14.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 120.0, M_EXTENDER_DO_NOT_CARE, // FULLY_EXTENDED,
                                0.0, true },
                            50,   0.0,    false },
   { 102,  M_ARM_TO_POS,    { (units::foot_t)14.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { -50.0, M_EXTENDER_DO_NOT_CARE,
                              -80.0, true },
                            50,   0.0,    false },
   { 103,  M_ARM_TO_POS,    { (units::foot_t)14.0,
                              (units::foot_t)-14.0,  // initial sharp 45 turn
                              (units::degree_t)0.0 },
                            { -55.0, M_EXTENDER_DO_NOT_CARE,
                                    -90.0, false },
                            50,   0.0,    false },
   { 104,  M_GO_TO_POSE,    { (units::foot_t)14.0,
                              (units::foot_t)-2.5,   // this is asymmetric;
                                                     // left side is different
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                                   -90.0, false },
                            0,   0.0,    false },
   { 105,  M_GO_TO_POSE,    { (units::foot_t)14.0,
                              (units::foot_t)4.0,     // should be 6.0
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                              -90.0, false },
                            0,   0.0,    false },
   { 106,  M_GO_TO_POSE,    { (units::foot_t)7.0,     // was 9.0
                              (units::foot_t)4.0,     // should be 6.0
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                              -90.0, false },
                            0,   0.0,    false },
   { 107,  M_BALANCE,       { (units::foot_t)7.0,     // was 9.0
                              (units::foot_t)4.0,     // should be 6.0
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE,
                                0.0, false },
                            0,   0.0,    false },
   { 108,  M_STOP,          { (units::foot_t)7.0,     // was 9.0
                              (units::foot_t)4.0,     // should be 6.0
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   { 109,  M_TERMINATE_SEQ, { (units::foot_t)7.0,     // was 9.0
                              (units::foot_t)4.0,     // should be 6.0
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                              0.0, false },
                            0,   0.0,    false },
   { 110,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   { 111,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   { 112,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   { 113,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   { 114,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   { 115,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   { 116,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   { 117,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   { 118,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },
   { 119,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)6.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, false },
                            0,   0.0,    false },

   // Index 120 autonomous sequence index (do nothing sequence):
   { 120,  M_STOP,          { (units::foot_t)0.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 140.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                                0.0, true },
                            0,   0.0,    false },
   { 121,  M_TERMINATE_SEQ, { (units::foot_t)0.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE, // FULLY_RETRACTED,
                              0.0, true },
                            0,   0.0,    false },
   { 122,  M_TERMINATE_SEQ, { (units::foot_t)0.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                              0.0, true },
                            0,   0.0,    false },
   { 123,  M_TERMINATE_SEQ, { (units::foot_t)0.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                              0.0, true },
                            0,   0.0,    false },
   { 124,  M_TERMINATE_SEQ, { (units::foot_t)0.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                              0.0, true },
                            0,   0.0,    false },
   { 125,  M_TERMINATE_SEQ, { (units::foot_t)0.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                              0.0, true },
                            0,   0.0,    false },
   { 126,  M_TERMINATE_SEQ, { (units::foot_t)0.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                              0.0, true },
                            0,   0.0,    false },
   { 127,  M_TERMINATE_SEQ, { (units::foot_t)0.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                              0.0, true },
                            0,   0.0,    false },
   { 128,  M_TERMINATE_SEQ, { (units::foot_t)0.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                              0.0, true },
                            0,   0.0,    false },
   { 129,  M_TERMINATE_SEQ, { (units::foot_t)0.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                              0.0, true },
                            0,   0.0,    false },

   // Index 130 autonomous sequence index (test sequence):
   // score, move forward (across the charging station) out of community,
   // wait, back, balance
   { 130,  M_ARM_TO_POS,    { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 120.0, M_EXTENDER_FULLY_RETRACTED,
                                0.0, true },
                            50,   0.0,    false },
   { 131,  M_ARM_TO_POS,    { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 120.0, M_EXTENDER_FULLY_EXTENDED,
                                0.0, true },
                            50,   0.0,    false },
   { 132,  M_ARM_TO_POS,    { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { -50.0, M_EXTENDER_DO_NOT_CARE,
                              -80.0, true },
                            50,   0.0,    false },
   { 133,  M_ARM_TO_POS,    { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { -55.0, M_EXTENDER_DO_NOT_CARE,
                                    -90.0, false },
                            50,   0.0,    false },
   { 134,  M_GO_TO_POSE,    { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                   -90.0, false },
                            0,   0.0,    false },
   { 135,  M_STOP,          { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                   -90.0, false },
                            0,   0.0,    false },
   { 136,  M_WAIT,          { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                   -90.0, false },
                            50,   0.0,    false },
   { 137,  M_GO_TO_POSE,    { (units::foot_t)3.0,     // should be 9.0
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            {  140.0, M_EXTENDER_FULLY_RETRACTED,
                                   0.0, false },
                            50,   0.0,    false },
   { 138,  M_BALANCE,       { (units::foot_t)3.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 140.0, M_EXTENDER_DO_NOT_CARE,
                                   0.0, false },
                            0,   0.0,    false },
   { 139,  M_STOP,          { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                   0.0, false },
                            0,   0.0,    false },
   { 140,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 0.0, M_EXTENDER_FULLY_RETRACTED,
                                   M_WRIST_FULLY_BACK, false },
                            0,   0.0,    false },
   { 141,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 0.0, M_EXTENDER_FULLY_RETRACTED,
                                   M_WRIST_FULLY_BACK, false },
                            0,   0.0,    false },
   { 142,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 0.0, M_EXTENDER_FULLY_RETRACTED,
                                   M_WRIST_FULLY_BACK, false },
                            0,   0.0,    false },
   { 143,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 0.0, M_EXTENDER_FULLY_RETRACTED,
                                   M_WRIST_FULLY_BACK, false },
                            0,   0.0,    false },
   { 144,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 0.0, M_EXTENDER_FULLY_RETRACTED,
                                   M_WRIST_FULLY_BACK, false },
                            0,   0.0,    false },
   { 145,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 0.0, M_EXTENDER_FULLY_RETRACTED,
                                   M_WRIST_FULLY_BACK, false },
                            0,   0.0,    false },
   { 146,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 0.0, M_EXTENDER_FULLY_RETRACTED,
                                   M_WRIST_FULLY_BACK, false },
                            0,   0.0,    false },
   { 147,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 0.0, M_EXTENDER_FULLY_RETRACTED,
                                   M_WRIST_FULLY_BACK, false },
                            0,   0.0,    false },
   { 148,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 0.0, M_EXTENDER_FULLY_RETRACTED,
                                   M_WRIST_FULLY_BACK, false },
                            0,   0.0,    false },
   { 149,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 0.0, M_EXTENDER_FULLY_RETRACTED,
                                   M_WRIST_FULLY_BACK, false },
                            0,   0.0,    false },

   

      // index 150: (previous) test autonomous: score, out, back, balance 
   { 150,  M_ARM_TO_POS,    { (units::foot_t)1.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 120.0, M_EXTENDER_FULLY_RETRACTED,
                                    20.0, true },
                            50,   0.0,    false },
   { 151,  M_ARM_TO_POS,    { (units::foot_t)0.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 120.0, M_EXTENDER_FULLY_EXTENDED,
                            // { 120.0, M_EXTENDER_DO_NOT_CARE,
                            // { 120.0, M_EXTENDER_FULLY_RETRACTED,
                                    20.0, true },
                            50,   0.0,    false },
   { 152,  M_ARM_TO_POS,    { (units::foot_t)9.5,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { -50.0, M_EXTENDER_DO_NOT_CARE,
                                    -80.0, true },
                            50,   0.0,    false },

   { 153,  M_ARM_TO_POS,    { (units::foot_t)9.5,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { -55.0, M_EXTENDER_DO_NOT_CARE,
                                    -90.0, false },
                            50,   0.0,    false },
   { 154,  M_GO_TO_POSE,    { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                   -90.0, false },
                            0,   0.0,    false },
   { 155,  M_STOP,          { (units::foot_t)15.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE,
                                   -90.0, false },
                            0,   0.0,    false },
   { 156,  M_WAIT,          { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE,
                                   -90.0, false },
                            50,   0.0,    false },
   { 157,  M_GO_TO_POSE,    { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            {  100.0, M_EXTENDER_DO_NOT_CARE,
                                   0.0, false },
                            50,   0.0,    false },
   { 158,  M_BALANCE,       { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_DO_NOT_CARE,
                                   0.0, false },
                            0,   0.0,    false },
   { 159,  M_STOP,          { (units::foot_t)9.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 100.0, M_EXTENDER_FULLY_RETRACTED,
                                   0.0, false },
                            0,   0.0,    false },
   { 160,  M_TERMINATE_SEQ, { (units::foot_t)9.0,
                              (units::foot_t)-6.0,
                              (units::degree_t)0.0 },
                            { 0.0, M_EXTENDER_FULLY_RETRACTED,
                                   M_WRIST_FULLY_BACK, false },
                            0,   0.0,    false },

   { 161,  M_TERMINATE_SEQ, { (units::foot_t)0.0,
                              (units::foot_t)0.0,
                              (units::degree_t)0.0 },
                            { 0.0, M_EXTENDER_FULLY_RETRACTED,
                                   M_WRIST_FULLY_BACK, false },
                            0,   0.0,    false },

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
      frc::Pose2d CurrentRobotPose;
      frc::Pose2d DesiredRobotPose;
      struct ArmPose  sArmPose;
      bool   teleop;
      double dLimelightDistanceToGoal;
      double dLimelightDesiredShooterSpeed;
   } sCurrState, sPrevState;

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
// #define DETECT_APRILTAGS 1
#undef DETECT_APRILTAGS 

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
      // camera.SetResolution(640, 480);                 // Set the resolution
      camera.SetResolution(320, 240);                    // Set the resolution
      camera.SetExposureManual(25);

                       // Get a CvSink. This will capture Mats from the Camera
      cs::CvSink cvSink = frc::CameraServer::GetVideo();
              // Setup a CvSource. This will send images back to the Dashboard
      cs::CvSource outputStream =
          // frc::CameraServer::PutVideo("DriveCam", 640, 480);
          frc::CameraServer::PutVideo("DriveCam", 320, 480);

      cv::Mat mat;     // Mats are very memory expensive. Lets reuse this Mat.
#ifdef DETECT_APRILTAGS
      cv::Mat grayMat;

      std::vector<int> tags;                               // Instantiate once
      cv::Scalar outlineColor = cv::Scalar(0, 255, 0);
      cv::Scalar crossColor = cv::Scalar(0, 0, 255);
#endif  // DETECT_APRILTAGS

      while (true)
      {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
         if (cvSink.GrabFrame(mat) == 0)
         {
                                           // Send error to the output stream.
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

               // std::cout << "Sideways: " << xid3 << " Depth: " << zid3
               //           << " Rotation: " << rotationid3 << std::endl;
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

       // Set all Arm position values (shoulder, extender, wrist, and grabber)
                              // Shoulder position (degrees, 0 is straight up)
             // The full range returned by GetPosition is 0 to -180, and that
             // is really ~300 degrees, so we multiply by 1.69 to make that
             // conversion.
             // When in base position, GetPosition() returns 0, and that is
             // really about +120.0 degrees, so we add that offset.
             // The result is a number in degrees, with 0 degrees straight up.
      sCurrState.sArmPose.dShoulderPosition = m_ShoulderEncoder.GetPosition()
                            * 1.69 + kShoulderPositionOffsetDeg;
                     // Don't set here; this value must be maintained when
                     // running the extender motor, because the photometric
                     // indicator shows black (false) for both FULLY_RETRACTED
                     // and FULLY_EXTENDED, and we only know which end has
                     // been hit because we know which direction we were
                     // moving (retracting or extending) when black was hit.
      // sCurrState.sArmPose.eExtenderPosition = 
                                // fully back (toward back robot) is 0;
                                // fully forward is 2306 (we measured this)
                                // NO: fully forward is 2306 (we measured this)
             // compute wrist position, relative to straight up (+ is forward,
             // once we invert the encoder report, since that is backward)
             // This depends on the arm position as well as the wrist motor.
      sCurrState.sArmPose.dWristPosition =
          -m_WristMotor.GetSelectedSensorPosition() * 210.0 / 2306.0 - 140.0 +
           sCurrState.sArmPose.dShoulderPosition;
      if ( 0 == iCallCount%500 ) {     // every 10 seconds
           cout << "WR: " << m_WristMotor.GetSelectedSensorPosition() << endl;
      }
                           // Don't set here; this value is maintained when we
                           // close or open the grabber.
      // sCurrState.sArmPose.bGrabberClosed = 

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
      m_motor.SetInverted(false);           // set forward direction of motor.

           /* Set limits to how much current will be sent through the motor */
#ifdef SAFETY_LIMITS
                       //  2 Amps below 5000 RPM, above 5000 RPM it ramps from
                       //  2 Amps down to  1 Amp  at 5700 RPM
      m_motor.SetSmartCurrentLimit( 2, 1, 5000);
#else
                       // 30 Amps below 5000 RPM, above 5000 RPM it ramps from
                       // 30 Amps down to 10 Amps at 5700 RPM
      m_motor.SetSmartCurrentLimit(30, 10, 5000);
#endif
      m_motor.GetForwardLimitSwitch(
                                rev::SparkMaxLimitSwitch::Type::kNormallyOpen)
                                     .EnableLimitSwitch(false);

                                          // Config 100% motor output to 12.0V
      m_motor.EnableVoltageCompensation(12.0);

                 // Set ramp rate (how fast motor accelerates or decelerates).
                 // We may have to try different RampRates here to
                 // eliminate chattering.
      m_motor.SetClosedLoopRampRate(0.2);
      m_motor.SetOpenLoopRampRate(0.2);

      // m_motor.SetIdleMode( rev::CANSparkMax::IdleMode::kCoast );
      m_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

      // The following call saves all settings permanently in the SparkMax's
      // flash memory, so the settings survive even through a brownout.
      // These statements should be uncommented for at least one deploy-enable
      // Roborio cycle after any of the above settings change, but they should
      // be commented out between changes, to keep from using all the
      // SparkMax's flash-write cycles (because it has a limited number
      // of flash-write cycles).
      // m_motor.BurnFlash();

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
      m_motor.SetSensorPhase(true);  // invert encoder value positive/negative
      m_motor.SetInverted(false);    // invert direction of motor itself.

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
      m_motor.ConfigClosedloopRamp(0.2);
      m_motor.ConfigOpenloopRamp(  0.2);

      m_motor.SetNeutralMode( NeutralMode::Brake );

   }      // MotorInitTalon()


     /* DriveToPose()  drive the robot to the specified pose (X,Y,rotation) */
     /* the first argument is the desired pose, and the second argument     */
     /* specifies whether the drive motors will be run; if the second       */
     /* argument is true, only the turning motors of the swerve will be     */
     /* run (that allows the swerve modules to orient themselves without    */
     /* moving the robot).                                                  */

   bool DriveToPose( frc::Pose2d DestinationPose,
                     bool bFreezeDriveMotors      )
   {
      static bool bReturnValue = true;
      static int iCallCount = 0;

      static double dBiggestX = 0.0,  dBiggestY = 0.0;
      static double dSmallestX = 0.0, dSmallestY = 0.0;
      frc::Transform2d transform = DestinationPose - 
                               m_swerve.m_poseEstimator.GetEstimatedPosition();
      dBiggestX = std::max( transform.X().value(), dBiggestX );
      dBiggestY = std::max( transform.Y().value(), dBiggestY );
      dSmallestX = std::min( transform.X().value(), dSmallestX );
      dSmallestY = std::min( transform.Y().value(), dSmallestY );
      if ( !bFreezeDriveMotors && 0 == iCallCount%100 ) {
         cout << "DTP() X/X, Y/Y, Rot: "
              << dSmallestX << "/" << dBiggestX << ", "
              << dSmallestY << "/" << dBiggestY << ", "
              << transform.Rotation().Degrees().value()
              << endl;
      }
// jag; 07apr2023    (Limiter --> LimiterA in 3 places)
      auto xSpeed = m_xspeedLimiterA.Calculate(
                        transform.X().value() / 2.0 ) * Drivetrain::kMaxSpeed;
      auto ySpeed = m_yspeedLimiterA.Calculate(
                        transform.Y().value() / 2.0 ) * Drivetrain::kMaxSpeed;
      auto rot =
             m_rotLimiterA.Calculate( transform.Rotation().Degrees().value() /
                                       180.00 ) * Drivetrain::kMaxAngularSpeed;

      xSpeed = std::min( Drivetrain::kMaxSpeed, xSpeed );
      ySpeed = std::min( Drivetrain::kMaxSpeed, ySpeed );
      rot    = std::min( Drivetrain::kMaxAngularSpeed, rot );
      xSpeed = std::max( -Drivetrain::kMaxSpeed, xSpeed );
      ySpeed = std::max( -Drivetrain::kMaxSpeed, ySpeed );
      rot    = std::max( -Drivetrain::kMaxAngularSpeed, rot );

//    if ( 4 == iCallCount%50 ) {
//       std::cout << "xSpeed: " << xSpeed.value() << std::endl;
//    }

      m_swerve.Drive( xSpeed, ySpeed, rot, true, bFreezeDriveMotors );

                    // If any (X/Y/Rotation) of the criteria for being
                    // at the desired pose (within 20 cm X and Y, and
                    // within 10 degrees yaw) are still not met...
      if ( ( transform.X() < (units::length::meter_t)-0.20 ) ||
           ( (units::length::meter_t)0.20 < transform.X()  ) ||
           ( transform.Y() < (units::length::meter_t)-0.20 ) ||
           ( (units::length::meter_t)0.20 < transform.Y()  ) ||
           ( transform.Rotation().Degrees() < (units::degree_t)-10.0 ) ||
           ( (units::degree_t)10.0 < transform.Rotation().Degrees()  )    ) {
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
      static int iFlatCount = 0;     // how many times has this function been
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
                                 // *has been flat* for at least 2.0 seconds.
      return bReturnValue && ( 100 < iFlatCount );  // was 25
   }  // DriveToBalance()


               // ArmToPosition() returns true when the arm is in the
               // desired position (shoulder within 10 degrees, etc.)
   bool ArmToPosition( struct ArmPose  sArmPose, bool bMoveGrabber )
   {
      bool bReturnValue = true;
      static int iCallCount = 0;

                                      // get local copies of desired positions
      auto dDesShoulder   = sArmPose.dShoulderPosition;
      auto eDesExtender   = sArmPose.eExtenderPosition;
      auto dDesWrist      = sArmPose.dWristPosition;
      auto bDesGrabberClosed = sArmPose.bGrabberClosed;
                                      // get local copies of current positions
      auto dCurShoulder   = sCurrState.sArmPose.dShoulderPosition;
      auto eCurExtender   = sCurrState.sArmPose.eExtenderPosition;
      auto dCurWrist      = sCurrState.sArmPose.dWristPosition;
      // bCurGrabberClosed is not used below.
      // auto bCurGrabberClosed = sCurrState.sArmPose.bGrabberClosed;
                 // Modify desired wrist angles to something that is possible,
                 // based on the current shoulder angle.  This allows us to
                 // know when we have reached as close as possible to our
                 // destination -- if we didn't do this, we could never
                 // reach an impossible destionation, and autonomous could
                 // get stuck waiting forever.
      if ( dDesWrist < dCurShoulder - 140.0 ) {
         dDesWrist = dCurShoulder - 140.0;
      } else if ( dCurShoulder + 90.0 < dDesWrist ) {
         dDesWrist = dCurShoulder + 90.0;
      }
                 // If asked to retract or extend, and the shoulder is past
                 // 125 degrees forward, raise it to 115 degrees, to clear
                 // the front of the octopus arms of the robot while
                 // retracting/extending
      if ( ( ( ( M_EXTENDER_FULLY_RETRACTED == eDesExtender ) &&
               ( M_EXTENDER_FULLY_RETRACTED != eCurExtender )    ) ||
             ( ( M_EXTENDER_FULLY_EXTENDED  == eDesExtender ) &&
               ( M_EXTENDER_FULLY_EXTENDED  != eCurExtender )    ) ) &&
           ( 125.0 < dDesShoulder ) ) {
         dDesShoulder = 115.0;
               // Don't let extender move until shoulder is in a safe position
         if ( 125.0 < dCurShoulder ) {
            eDesExtender = M_EXTENDER_DO_NOT_CARE;
         }
         bReturnValue = false;  // Don't allow this function to say it is
                                // complete until condition is no longer true
                                // (extender is fully retracted or extended).
      }
            // If up near the top of the arc, move the wrist flat temporarily,
            // to prevent going above the height limit.
      if ( ( -45.0 < dCurShoulder ) && ( dCurShoulder < 45.0  ) ) {
                              // if we need to go over the top (back to front)
         if ( ( dCurShoulder < -5.0 ) && ( 5.0 < dDesShoulder ) ) {
                                       // if wrist is already pointing forward
            if ( dCurShoulder < dCurWrist - 10.0 ) {
               dDesWrist = 90.0;       // point it flat forward
            } else {
               dDesWrist = -90.0;      // else make it flat backwards
            }
            bReturnValue = false;    // Don't allow function to say it is done

                         // else if we need to go over the top (front to back)
         } else if ( ( dDesShoulder < 0.0 ) && ( 0.0 < dCurShoulder ) ) {
                               // if wrist is already pointing forward
                               // (in the direction of travel toward the back)
            if ( dCurWrist + 10.0 < dCurShoulder ) {
               dDesWrist = -90.0;   // point it flat toward back of robot
            } else {
               dDesWrist =  90.0;   // else make it flat toward front of robot
            }
            bReturnValue = false;    // Don't allow function to say it is done
         }
      }
            // If in the top semicircle, don't try to extend the extender.
      if ( ( -90.0 < dCurShoulder ) && ( dCurShoulder < 90.0  ) ) {
         if ( M_EXTENDER_FULLY_EXTENDED == eDesExtender ) {
            eDesExtender = M_EXTENDER_DO_NOT_CARE;
            // Do not set bReturnValue = false here, so the maneuver can
            // complete even if the arm is not fully extended.
         }
      }
            // If in the front and angled down, and must extend extender...
      if ( ( 90.0 < dCurShoulder ) &&
           ( M_EXTENDER_FULLY_EXTENDED == eDesExtender ) &&
           ( M_EXTENDER_FULLY_EXTENDED != eCurExtender )     ) {
               // then don't allow it to rise above 110 degrees until extended
            dDesShoulder = std::max( 110.0, dDesShoulder );
            bReturnValue = false;    // Don't allow function to say it is done

            // else if in the back and angled down, and must extend extender...
      } else if ( ( dCurShoulder < -90.0 ) &&
                  ( M_EXTENDER_FULLY_EXTENDED == eDesExtender ) &&
                  ( M_EXTENDER_FULLY_EXTENDED != eCurExtender )     ) {
               // then don't allow it to rise above 110 degrees until extended
            dDesShoulder = std::min( -110.0, dDesShoulder );
            bReturnValue = false;    // Don't allow function to say it is done
      }

      double dShoulderGravityCorrection =
                       -0.03 * sin( dCurShoulder * std::numbers::pi / 180.0 );
      double dWristGravityCorrection =
                       -0.13 * sin( dCurWrist    * std::numbers::pi / 180.0 );

                // compute power to send through shoulder motor (+ is forward)
                // 4 degrees out of position results in 1 volt forward, minus
                // any arm speed correction. After multiplying the arm speed
                // by 1.69/60.0, the result is in degrees/second.  Dividing
                // that by 4.0 means we'll get one volt subtracted from the
                // arm drive force when it's moving at 4 degrees/second.
         //  At the end of the GetVelocity() term;
         //  /4.0 is jerky. 40.0 no effect, 8.0 is smooth, 16.0 OK too.
      double dShoulderSpeed = ( dDesShoulder - dCurShoulder )/4.0
                       - m_ShoulderEncoder.GetVelocity() * 1.69 / 60.0 / 8.0;
      dShoulderSpeed = std::max( dShoulderSpeed, -12.0 );
      dShoulderSpeed = std::min( dShoulderSpeed,  12.0 );

      if ( 0 == iCallCount%10000 ) {
         cout << "ArmToPos: shldrspd: " << dShoulderSpeed << endl;
      }

      m_ShoulderMotor.SetVoltage( units::volt_t{ dShoulderSpeed +
                                               dShoulderGravityCorrection } );

      if ( 10.0 < abs( dCurShoulder - dDesShoulder )  ) {
         bReturnValue = false;
      }

             // If the photometric indicator ever indicates true (no black
             // is seen) then we know the extender is in the middle somewhere.
      if ( extenderForwardReverseLimitDIO2.Get() ) {
         sCurrState.sArmPose.eExtenderPosition = M_EXTENDER_MIDDLE;
      }
                                          // if we need to extend the extender
      if ( ( M_EXTENDER_FULLY_EXTENDED == eDesExtender ) &&
           ( M_EXTENDER_FULLY_EXTENDED != eCurExtender ) ) {
                                    // extend the extender (motor forward)
         m_ExtenderMotor.SetVoltage( units::volt_t{ 12.0 } );
         bReturnValue = false;
                           // Once we see the black mark, record that position
         if ( ( M_EXTENDER_MIDDLE == eCurExtender ) &&
              !extenderForwardReverseLimitDIO2.Get()   ) {
            eCurExtender = M_EXTENDER_FULLY_EXTENDED;
            sCurrState.sArmPose.eExtenderPosition = M_EXTENDER_FULLY_EXTENDED;
         }
                                    // else if we need to retract the extender
      } else if (
               ( M_EXTENDER_FULLY_RETRACTED == eDesExtender ) &&
               ( M_EXTENDER_FULLY_RETRACTED != eCurExtender ) ) {
                                    // retract the extender (motor backwards)
         m_ExtenderMotor.SetVoltage( units::volt_t{ -12.0 } );
         bReturnValue = false;
                           // Once we see the black mark, record that position
         if ( ( M_EXTENDER_MIDDLE == eCurExtender ) &&
              !extenderForwardReverseLimitDIO2.Get()       ) {
            eCurExtender = M_EXTENDER_FULLY_RETRACTED;
            sCurrState.sArmPose.eExtenderPosition = M_EXTENDER_FULLY_RETRACTED;
         }
      } else {
         m_ExtenderMotor.SetVoltage( units::volt_t{ 0.0 } );
      }

                   // compute power to send through wrist motor (+ is forward)
                   // jag; 30mar2023 -- I reduced the denominator of the next
                   // expression from 2.0 to 1.0 to double the power of the
                   // wrist motor, because the extra friction from the new
                   // wrist means it could never reach a setpoint.
                   // It was about 15 degrees off all the time, but with this
                   // change it is only off about 6 degrees now.
                   // But it'll be much better to eliminate that friction!
// jag; 07apr2023    (  /1.0 --> /2.0 )
      double dWristSpeed = ( dDesWrist - dCurWrist )/2.0;   // jag; was /2.0
      // double dWristSpeed = ( dDesWrist - dCurWrist )/1.0;   // jag; was /2.0
      dWristSpeed = std::max( dWristSpeed, -12.0 );
      dWristSpeed = std::min( dWristSpeed,  12.0 );

                 // If we need to move the wrist back toward the back of robot
      if ( ( dDesWrist + 0.0 < dCurWrist ) &&
           ( wristForwardLimitDIO0.Get() ) ) {
         m_WristMotor.SetVoltage( units::volt_t{ dWristSpeed +
                                                  dWristGravityCorrection } );
                 // else if we need to move the wrist forward toward the
                 // front of robot
      } else if ( ( dCurWrist < dDesWrist - 0.0 ) &&
                  ( wristReverseLimitDIO1.Get() ) ) {
         m_WristMotor.SetVoltage( units::volt_t{  dWristSpeed +
                                                  dWristGravityCorrection } );
                 // else the wrist is where we want it
      } else {
              // this should seldom happen (only when limit switches tripped)
      // m_WristMotor.SetVoltage( units::volt_t{ 0.0 } );        // stop motor
         m_WristMotor.SetVoltage( units::volt_t{  dWristGravityCorrection } );
      }
      if ( 10.0 < abs( dCurWrist - dDesWrist )  ) {
         bReturnValue = false;
      }

      if ( bMoveGrabber ) {
         if ( bDesGrabberClosed ) {
                                                        // close the grabber
            m_grabberPortSolenoid.Set( frc::DoubleSolenoid::Value::kForward);
            m_grabberStbdSolenoid.Set( frc::DoubleSolenoid::Value::kForward);
            // bCurGrabberClosed = true;   // never used
            sCurrState.sArmPose.bGrabberClosed = true;
         } else {
                                                        // open the grabber
            m_grabberPortSolenoid.Set( frc::DoubleSolenoid::Value::kReverse);
            m_grabberStbdSolenoid.Set( frc::DoubleSolenoid::Value::kReverse);
            // bCurGrabberClosed = false;   // never used
            sCurrState.sArmPose.bGrabberClosed = false;
         }
      }   // if ( bMoveGrabber )
      if ( 0 == iCallCount%100 ) {
         cout << "ArmToPos: shldr: "
              << sArmPose.dShoulderPosition << "/" 
              << sCurrState.sArmPose.dShoulderPosition
              << "  ext: " << sArmPose.eExtenderPosition << "/"
              << sCurrState.sArmPose.eExtenderPosition
              << "  wrist: " << sArmPose.dWristPosition << "/"
              << sCurrState.sArmPose.dWristPosition
              << "  grabber: " << sArmPose.bGrabberClosed << "/"
              << sCurrState.sArmPose.bGrabberClosed << endl;
      }

      if ( bReturnValue ) {
         // cout << "ArmToPosition() returning true." << endl;
      }

      iCallCount++;
                        // Only return true if all movement points of the arm
                        // are within range of the desired settings.
      return bReturnValue;
   }  // ArmToPosition()

               // ArmByJoysticks() allows controlling the arm joints
               // with joysticks.
   bool ArmByJoysticks( void )
   {
      bool bReturnValue = true;
      static int iCallCount = 0;

             bool bXButton = m_OperatorController.GetXButton();
      static bool bXButton_prev = false;

      static double dArmDirection = 1.0;

      static double dWristSpeed = 0.0;

      if ( !bXButton_prev && bXButton ) {
         if ( dArmDirection < 0.0 ) {
            dArmDirection =  1.0;
         } else {
            dArmDirection = -1.0;
         }
      }
      bXButton_prev = bXButton;

                                    // Run the shoulder motor to rotate the arm
                  // first calculate the gravity correction amount
      double dShoulderGravityCorrection =
                        -0.03 * sin( sCurrState.sArmPose.dShoulderPosition *
                                                   std::numbers::pi / 180.0 );
      if ( 2 == iCallCount%50000 ) {
         cout << "Shoulder pos: " << sCurrState.sArmPose.dShoulderPosition
              << " Shoulder corr: " << dShoulderGravityCorrection
              << endl;
      }

      m_ShoulderMotor.SetVoltage(
              units::volt_t{ dShoulderGravityCorrection +
                            frc::ApplyDeadband( dArmDirection *
                                              m_OperatorController.GetRightY(),
                                                0.10) } * 12.0 );

                                    // Run the extender motor to extend the arm
      if ( extenderForwardReverseLimitDIO2.Get() ) {
         sCurrState.sArmPose.eExtenderPosition = M_EXTENDER_MIDDLE;
      }
      if ( 0.10 < m_OperatorController.GetLeftTriggerAxis() ) {
              // if not yet fully retracted,
         if ( ( M_EXTENDER_FULLY_RETRACTED !=
                                   sCurrState.sArmPose.eExtenderPosition ) ) {

            m_ExtenderMotor.SetVoltage(
               units::volt_t{
           -frc::ApplyDeadband(m_OperatorController.GetLeftTriggerAxis(),
                               0.10) } * 8.0 );        // increase to * 12.0 ?
                           // Once we see the black mark, record that position
            if ( ( M_EXTENDER_MIDDLE == 
                                sCurrState.sArmPose.eExtenderPosition ) &&
                 !extenderForwardReverseLimitDIO2.Get()                    ) {
               sCurrState.sArmPose.eExtenderPosition =
                                                   M_EXTENDER_FULLY_RETRACTED;
            }
              // else if the override button is pressed
         } else if ( m_OperatorController.GetYButton() ) {
            m_ExtenderMotor.SetVoltage( units::volt_t{ -3.50 } );
         } else {
                // as a fail-safe, we could allow pushing at a very low power,
                // in case the extenderForwardReverseLimitDIO2 sensor failed.
                // m_ExtenderMotor.SetVoltage( units::volt_t{ -3.50 } );
            // NO: we have instead created an override button (controller "A")
            // so no motor movement at all here once the limit is reached.
            m_ExtenderMotor.SetVoltage( units::volt_t{ 0.0 } );
         }
      } else if ( 0.10 < m_OperatorController.GetRightTriggerAxis() ) {

         if ( M_EXTENDER_FULLY_EXTENDED !=
                                     sCurrState.sArmPose.eExtenderPosition ) {
            m_ExtenderMotor.SetVoltage(
               units::volt_t{
            frc::ApplyDeadband(m_OperatorController.GetRightTriggerAxis(),
                               0.10) } * 8.0 );
                           // Once we see the black mark, record that position
            if ( ( M_EXTENDER_MIDDLE ==
                                 sCurrState.sArmPose.eExtenderPosition ) &&
                 !extenderForwardReverseLimitDIO2.Get()                     ) {
               sCurrState.sArmPose.eExtenderPosition =
                                                     M_EXTENDER_FULLY_EXTENDED;
            }
         } else if ( m_OperatorController.GetYButton() ) {
            m_ExtenderMotor.SetVoltage( units::volt_t{ 3.50 } );
         } else {
                 // as a fail-safe, allow pushing at a very low power,
                 // in case the extenderForwardReverseLimitDIO2 sensor failed.
            // m_ExtenderMotor.SetVoltage( units::volt_t{ 3.50 } );
            // NO: Brian had me remove the fail-safe.
            m_ExtenderMotor.SetVoltage( units::volt_t{ 0.0 } );
         }
      } else {
         m_ExtenderMotor.SetVoltage( units::volt_t{ 0.0 } );
      }
                      // Run the wrist motor to rotate the wrist (and grabber)
      double dWristGravityCorrection =
                        -0.13 * sin( sCurrState.sArmPose.dWristPosition *
                                                   std::numbers::pi / 180.0 );

      if ( 0 == iCallCount%5000 ) {
         cout << "wrst: " << sCurrState.sArmPose.dWristPosition << endl;
      }
      dWristSpeed = dArmDirection * m_OperatorController.GetLeftY();
      if ( ( wristForwardLimitDIO0.Get() && ( dWristSpeed <= 0.0 ) ) ||
           ( wristReverseLimitDIO1.Get() && ( 0.0 <= dWristSpeed ) )    ) {
         m_WristMotor.SetVoltage(
           units::volt_t{  dWristGravityCorrection +
            frc::ApplyDeadband( dWristSpeed, 0.10) } *
#ifdef SAFETY_LIMITS
                                       6.0 );
#else
                                       12.0 );
#endif
      } else {
         m_WristMotor.SetVoltage( units::volt_t{ 0.0 } );
      }

      if ( 0 == iCallCount%100 ) {
         cout << "ArmByJoy: shldr: "
              << sCurrState.sArmPose.dShoulderPosition
              << "  ext: " << sCurrState.sArmPose.eExtenderPosition
              << "  wrist: " << sCurrState.sArmPose.dWristPosition
              << "  grabber: " << sCurrState.sArmPose.bGrabberClosed << endl;
      }

      iCallCount++;

      return bReturnValue;
   }  // ArmByJoysticks()


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

         // Get the x speed. We are inverting this because Xbox controllers
         // return negative values when we push forward.
      const auto xSpeed = -m_xspeedLimiter.Calculate(
                      frc::ApplyDeadband(m_DriveController.GetLeftY(), 0.10)) *
                          Drivetrain::kMaxSpeed * dSpeedFactor;

         // Get the y speed or sideways/strafe speed. We are inverting this
         // because we want a positive value when we pull to the left.
         // Xbox controllers return positive values when you pull to the right
         // by default.
      const auto ySpeed = -m_yspeedLimiter.Calculate(
                      frc::ApplyDeadband(m_DriveController.GetLeftX(), 0.10)) *
                          Drivetrain::kMaxSpeed * dSpeedFactor;

         // Get the rate of angular rotation. We are inverting this because 
         // we want a positive value when we pull to the left (remember:
         // CCW is positive in mathematics). Xbox controllers return positive
         // values when you pull to the right by default.
      auto rot = -m_rotLimiter.Calculate(
                    frc::ApplyDeadband(m_DriveController.GetRightX(), 0.10)) *
                       Drivetrain::kMaxAngularSpeed * dSpeedFactor;

                   // If the driver is pulling the right controller backward,
                   // slow down the rotation rate (pulled all the way back
                   // would reduce the rotation rate by a factor of 0.01,
                   // pulling it just beyond halfway would be a factor of 0.5)
      if ( 0.5 < m_DriveController.GetRightY() ) {
         rot = rot * ( 1.01 - m_DriveController.GetRightY() );
      }
      if ( m_DriveController.GetLeftBumper() ) {
         rot = -Drivetrain::kMaxAngularSpeed;
      } else if ( m_DriveController.GetRightBumper() ) {
         rot =  Drivetrain::kMaxAngularSpeed;
      }
#ifdef JAG_NOTDEFINED
           // The code here implements an idea of Alice.
           // It gives the driver the ability to orient the robot yaw
           // immediately in one of the cardinal directions, with the
           // Driver's POV buttons, which return 0, 45, 90, 135, 180, etc.
      int iDriversPOV = m_DriveController.GetPOV();
      if ( -1 < iDriversPOV ) {
         int iCurrYaw =
                     sCurrState.CurrentRobotPose.Rotation().Degrees().value();
         if ( ( iDriversPOV - iCurrYaw < -180 ) ||
              (  180 < iDriversPOV - iCurrYaw )    ) {
                            // turn left
            rot = m_rotLimiter.Calculate(  ( iDriversPOV - iCurrYaw ) /
                                       180.00 ) * Drivetrain::kMaxAngularSpeed;
         } else {
                            // turn right
            rot = m_rotLimiter.Calculate( -( iDriversPOV - iCurrYaw ) /
                                       180.00 ) * Drivetrain::kMaxAngularSpeed;
         }
         rot = std::min( Drivetrain::kMaxAngularSpeed, rot );
         rot = std::max( -Drivetrain::kMaxAngularSpeed, rot );
      }
#endif

              // If "B" button is pressed, don't allow the drive motors to move
      m_swerve.Drive( xSpeed, ySpeed, rot, fieldRelative,
                      m_DriveController.GetBButton() );

   }  // DriveWithJoystick()


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
                                            (units::degree_t)0.0 },
                                          { 140.0, M_EXTENDER_FULLY_RETRACTED,
                                            0.0, false },
                                          0, 0.0, false };
      // static int icmdSeqManeuverCallCount = 0;
      static double dWaitCount =  50.0; // ticks to wait (50 ticks = 1 second)

      switch ( mSeq.type )
      {
      case M_TERMINATE_SEQ:
                                         // Make sure drive motors are stopped
         m_swerve.Drive( (units::velocity::meters_per_second_t)0.0,
                         (units::velocity::meters_per_second_t)0.0,
                         (units::angular_velocity::radians_per_second_t)0.0,
                         false );   // Robot-centric drive, not field-oriented
         m_ShoulderMotor.SetVoltage( units::volt_t{ 0.0 } );
         m_ExtenderMotor.SetVoltage( units::volt_t{ 0.0 } );
         m_WristMotor.SetVoltage( units::volt_t{ 0.0 } );
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
         {
            frc::Pose2d currPose =
                               m_swerve.m_poseEstimator.GetEstimatedPosition();
            cout << "EM: STOP completed, final X/Y/Rot: ";
            cout << currPose.X().value() << "/" << currPose.Y().value() <<
                    "/" << currPose.Rotation().Degrees().value() << endl;
         }
         m_WristMotor.SetVoltage( units::volt_t{ 0.0 } );
         bRetVal = true;                  // and exit this maneuver immediately
         break;

      case M_GO_TO_POSE:
         if ( mSeqPrev.index != mSeq.index ) { // first call to this maneuver?
            dWaitCount = 10.0;           // yes; store number of ticks to wait
         } else {                  // (we wait so swerves can orient properly)
            dWaitCount -= 1.0;
         }
         // Drive straight to the specified pose.
         // The DriveToPose() function returns true when it is near the pose.
         // While the second argument is true, the drive motors won't run;
         // this is necessary to fix lurch at the beginning of each maneuver.
         bRetVal = DriveToPose( mSeq.DestinationPose,
                                0.0 < dWaitCount );
                                            // Move the arm if necessary, too.
         ArmToPosition( mSeq.sArmPose, true );

                                           // If we have driven far enough...
         if ( bRetVal ) {
            frc::Pose2d currPose =
                               m_swerve.m_poseEstimator.GetEstimatedPosition();
            cout << "EM: DriveToPose() completed, final X/Y/Rot: ";
            cout << currPose.X().value() << "/" << currPose.Y().value() <<
                    "/" << currPose.Rotation().Degrees().value() << endl;
         }
         break;

      case M_ARM_TO_POS:
         if ( mSeqPrev.index != mSeq.index ) { // first call to this maneuver?
            dWaitCount = 10.0;           // yes; store number of ticks to wait
         } else {
            dWaitCount -= 1.0;
         }
         bRetVal = ArmToPosition( mSeq.sArmPose, true );
                     // align swerve modules for next destination (see below).
         DriveToPose( mSeq.DestinationPose, true );
                     // prevent going to next maneuver immediately
                     // (this is to prevent the robot from moving while the
                     //  grabber is opening, for example).
         if ( bRetVal && 0.0 < dWaitCount ) {
            bRetVal = false;
         }
         if ( bRetVal ) {
            cout << "EM: ArmToPos() completed, continuing..." << endl;
         }
            // we should never take longer than 4 seconds to move the arm
            // so continue anyway, even if some criteria not yet met.
            // This protects against a failed extension, for example;
            // without this, this maneuver would wait forever for the
            // extender to go all the way out.
         if ( dWaitCount < -200.0 ) {
            cout << "EM: ArmToPos() timed out, continuing..." << endl;
            bRetVal = true;
         }

         break;

      case M_WAIT:
         if ( mSeqPrev.type != mSeq.type ) {          // first call to M_WAIT?
                                         // yes; store number of ticks to wait
            dWaitCount = (int)mSeq.iArg;
         } else {
            dWaitCount -= 1.0;
         }
                   // align the swerve modules as if we were going to drive
                   // to the specified pose, but don't actually drive anywhere
                   // (don't run the drive motors in the swerve module).
                   // This can be used to initialize the swerves -- to wait
                   // until they lock into the correct turn directions.
         DriveToPose( mSeq.DestinationPose, true );
                   // Maintain the arm position, too.
         ArmToPosition( mSeq.sArmPose, true );
                  // If we have waited long enough,
         if ( dWaitCount <= 0.0 ) {
            cout << "EM: M_WAIT completed" << endl;
            bRetVal = true;          // done waiting; change to next maneuver.
         } else {
            bRetVal = false;         // stay in this maneuver; keep waiting.
         }
         break;

      case M_BALANCE:
         // Drive to balance point (where robot is flat on charging station).
         // The DriveToBalance() function returns true when it is balanced.
         bRetVal = DriveToBalance();
         ArmToPosition( mSeq.sArmPose, true );

                          // If we are balanced (flat on the charging station)
         if ( bRetVal ) {
            frc::Pose2d currPose =
                               m_swerve.m_poseEstimator.GetEstimatedPosition();
            cout << "EM: DriveToBalance() completed, final X/Y/Rot: ";
            cout << currPose.X().value() << "/" << currPose.Y().value() <<
                    "/" << currPose.Rotation().Degrees().value() << endl;
         }
         break;

      case M_JUMP:
         bRetVal = true;    // Just return true, and the function which called
                            // this should go to the next specified maneuver.

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


      /*---------------------------------------------------------------------*/
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
      static int iCallCount = 0;
      // static int icmdSeqManeuverCallCount = 0;

      if ( 0 == iCallCount%150 ) {
         cout << "Maneuver Index: " << maneuverIndex << endl;
      }

                     // execute the current maneuver, and if it is finished...
      if ( executeManeuver( mSeq[ maneuverIndex ] ) ) {
                // if this is a JUMP command, change to the specified maneuver
         if ( M_JUMP == mSeq[maneuverIndex].type ) {
            maneuverIndex = mSeq[maneuverIndex].iArg;
         } else {        // otherwise just go to the next maneuver in sequence

            maneuverIndex++;                            // go to next maneuver
         }
         cout << "New Maneuver Index: " << maneuverIndex << endl;
      }
      iCallCount++;
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

         ArmMotorInitSpark( m_ShoulderMotor );
         ArmMotorInitTalon( m_ExtenderMotor );
         ArmMotorInitTalon( m_WristMotor );

      // Set the distance per pulse for the drive encoder. We don't know or
      // care if the distance reported by the m_ShoulderEncoder is correct;
      // we only care that the kShoulderEncoderMin and kShoulderEncoderMax
      // const values are correct, for the limits of the shoulder joint; and
      // we measured those on the robot.
         m_ShoulderEncoder.SetPositionConversionFactor( 1.0 / 1.0 );
         m_ShoulderEncoder.SetVelocityConversionFactor( 1.0 / 1.0 );
         m_ShoulderMotor.SetSoftLimit(
                               rev::CANSparkMax::SoftLimitDirection::kForward,
                               kShoulderEncoderMax );   // eg: 15
         m_ShoulderMotor.SetSoftLimit(
                               rev::CANSparkMax::SoftLimitDirection::kReverse,
                               kShoulderEncoderMin );   // eg: 0
         m_ShoulderMotor.EnableSoftLimit(
                               rev::CANSparkMax::SoftLimitDirection::kForward,
                               true );
         m_ShoulderMotor.EnableSoftLimit(
                               rev::CANSparkMax::SoftLimitDirection::kReverse,
                               true );
      // m_ShoulderMotor.BurnFlash();  // Do this whenever the config changes.

         m_ExtenderMotor.ConfigPeakOutputForward(  1.0, 10 );
         m_ExtenderMotor.ConfigPeakOutputReverse( -1.0, 10 );
         m_WristMotor.ConfigPeakOutputForward(  1.0, 10 );
         m_WristMotor.ConfigPeakOutputReverse( -1.0, 10 );
         m_WristMotor.ConfigReverseSoftLimitThreshold( kWristEncoderMin, 0 );
         m_WristMotor.ConfigForwardSoftLimitThreshold( kWristEncoderMax, 0 );
//         m_WristMotor.ConfigReverseSoftLimitEnable( true, 0 );
//         m_WristMotor.ConfigForwardSoftLimitEnable( true, 0 );

               // record initial arm pose (shoulder, extender, wrist, grabber)
         sCurrState.sArmPose.dShoulderPosition = kShoulderPositionOffsetDeg;
         sCurrState.sArmPose.eExtenderPosition = M_EXTENDER_FULLY_RETRACTED;
         sCurrState.sArmPose.dWristPosition = 0.0; // M_WRIST_FULLY_BACK;
         sCurrState.sArmPose.bGrabberClosed = true;
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
      // if (m_DriveController.GetAButton())
      if ( 0.9 < m_DriveController.GetLeftTriggerAxis() )
      {
         dSpeedFactor = 0.1;
      } else {
         dSpeedFactor = 1.0;
      }

      if (m_DriveController.GetLeftBumperPressed())
      {
         pipelineindex--;
         if (pipelineindex < 0)
         {
            pipelineindex = 3;
         }
      }
      if (m_DriveController.GetRightBumperPressed())
      {
         pipelineindex++;
         if (pipelineindex > 3)
         {
            pipelineindex = 0;
         }
      }
      if (m_DriveController.GetYButtonPressed())
      {
         ldriver = !ldriver;
      }

      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->
                                       PutNumber( "pipeline", pipelineindex );
      // 0 = Vision processing mode, lights auto
      // 1 = Drivermode, lights off
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->
                                       PutNumber( "camMode", ldriver );
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->
// jag; 07apr2023    ( turn Limelight LEDs off always )
#ifndef LIMELIGHT_ACTIVATELEDS
                                       PutNumber( "ledMode", ldriver );
#else
                                       PutNumber( "ledMode", 1 ); // LEDs off
#endif

      std::vector<double> lcam_pose_target =
                   limenttable->GetNumberArray("tid", std::vector<double>(6));

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
      //   m_compressor.Disable();   // For debug/testing only.
      m_compressor.EnableDigital();
   }  // TestInit()

      /*---------------------------------------------------------------------*/
      /* TestPeriodic()                                                      */
      /* This function is called every 20 milliseconds, as long as the robot */
      /* is in Test mode.                                                    */
      /* In this mode the Roborio cannot drive any motors, but it can read   */
      /* the joystick, joystick/console buttons, and sensors.                */
      /*---------------------------------------------------------------------*/
   void TestPeriodic() override {
      static int  iCallCount = 0;

      if ( 0 == iCallCount%250 ) {    // every 5 seconds
         cout << m_ShoulderEncoder.GetPosition() << endl;
      }
              // For testing (reporting gyro settings).
      m_swerve.DriveUphill( (units::velocity::meters_per_second_t)0.0 );

      iCallCount++;
   }  // TestPeriodic()


      /*---------------------------------------------------------------------*/
      /* AutonomousInit()                                                    */
      /* This function is called once when the robot enters Autonomous mode. */
      /*---------------------------------------------------------------------*/
   void AutonomousInit() override {
      RobotInit();
      //   m_compressor.Disable();   // For debug/testing only.
      m_compressor.EnableDigital();
      iCallCount = 0;
      m_swerve.Reset();

      GetAllVariables();
      sCurrState.teleop = false;

                                   // mSeqIndex can be set to different values,
                                   // based on the console switches.
             // Button switch 1 means the robot is starting at the left side,
             // button switch 2 means the robot is starting in the middle, and
             // button switch 3 means the robot is starting at the right side.
             // Button switch 4 specifies to try to continue onto the charging
             // station, and balance.
             // Special cases for testing: button switch 1 and 3 together mean
             // do nothing (just stop).
             // Button switches 1,2,3, and 4 (all switches on) means do a test
             // auto (for example, immediately start balancing on the charging
             // station).
      if ( BUTTON_SWITCH1 && BUTTON_SWITCH2 &&
           BUTTON_SWITCH3 && BUTTON_SWITCH4 ) {
         mSeqIndex = 130;         // Test auto (forward, wait, back) 
      } else if ( BUTTON_SWITCH1 && BUTTON_SWITCH3 ) {
         mSeqIndex = 120;         // Do nothing auto (stop, end) 
      } else if ( BUTTON_SWITCH1 && BUTTON_SWITCH4 ) {
         mSeqIndex = 60;         // (score, forward left, right, back, balance)
      } else if ( BUTTON_SWITCH2 && BUTTON_SWITCH4 ) {
         mSeqIndex = 80;         // (score, forward straight, back, balance)
      } else if ( BUTTON_SWITCH3 && BUTTON_SWITCH4 ) {
         mSeqIndex = 100;        // (score, forward right, left, back, balance)
      } else if ( BUTTON_SWITCH1 ) {
         mSeqIndex =  0;         // (score, forward left, try to grab cube,
                                 // then back and score again.
// jag; 07apr2023 (possible way to add code to account for cable guard)
                                 // We could add code here like this:
         // if ( frc::DriverStation::kRed ==
         //                               frc::DriverStation::GetAlliance() ) {
                           // adjust some of the maneuvers, to account for
                           // the disruption of the cable guard
                           // (add 6" to each drive distance).
                           // (The lines below aren't correct; they need to
                           //  be whatever it takes to add 0.5 to each X field)
         //   frc::Transform2d transformIncrement =
         //                            (frc::Transform2d){ { 0.5, 0.0 }, 0.0 };
         //    mSeq[5].DestinationPose =
         //                       mSeq[5].DestinationPose + transformIncrement;
         //    mSeq[6].DestinationPose.X += 0.5;   etc...
         // }
      } else if ( BUTTON_SWITCH2 ) {
         mSeqIndex = 20;         // score, forward straight, continue facing
                                 // forward
      } else if ( BUTTON_SWITCH3 ) {
         mSeqIndex = 40;         // score, forward right, stop
      } else {
         mSeqIndex =  120;  // TEMPORARY: do nothing (for shop safety)
//       mSeqIndex =  80;  // DEFAULT auto sequence: no switch flipped, so
                           // do default auto, which as of now is the middle
                           // robot position, that also tries to balance
                           // (same as switch2 and switch4).
           // We changed to this as the default because we had some problems
           // with the robot not always performing the middle auto at
           // competition (it just sat dead until the end of auto period), and
           // we eventually found (after the usual chorus, blaming it on the
           // code) the reason was that the driver station was not always
           // reporting BUTTON_SWITCH2 correctly.  Leaving this as the
           // default will work around that problem, if it happens again --
           // and it won't cause a problem, no matter where the robot starts
           // auto, because all it will do (in case the driver station fails
           // to report the switches) is make the robot go forward, then back;
           // so it won't conflict with with another robot, whether our robot
           // starts on the left, middle, or right.
      }
   }   // AutonomousInit()

   void AutonomousPeriodic() override
   {
      GetAllVariables();

                            // Perform a sequence of maneuvers, transitioning
                            // to next maneuver in the sequence when necessary.
      mSeqIndex = executeManeuverSequence( mSeqIndex );
      m_swerve.UpdateOdometry();

      return;
   }   // AutonomousPeriodic()


   void TeleopInit() override
   {
      GetAllVariables();

      //   m_compressor.Disable();   // For debug/testing only.
      m_compressor.EnableDigital();
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

      GetAllVariables();

      if ( 0 == iCallCount%50000 ) {
         cout << "Wrist Enc Vel: "
              << m_WristMotor.GetSelectedSensorVelocity()
              << " position: "
              << m_WristMotor.GetSelectedSensorPosition() << endl;
      }
      m_swerve.UpdateOdometry();

      DriveWithJoystick(true);

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

      int iConPOV = m_OperatorController.GetPOV();

      if ( m_OperatorController.GetAButton() ) {
         ArmToPosition( M_ARMPOS_HOME, false );   // Go to home position,
                                                  // without modifying grabber
      } else if (   0 == iConPOV ) {
         ArmToPosition( M_ARMPOS_HICONE, false ); // high cone scoring position
      } else if (  45 == iConPOV ) {
         ArmToPosition( M_ARMPOS_HUMNPLYRSTATN, false ); // human player statn
      } else if (  90 == iConPOV ) {
         ArmToPosition( M_ARMPOS_FRONTGRAB, false ); // Low front pickup
      } else if ( 180 == iConPOV ) { 
         ArmToPosition( M_ARMPOS_LOCONE, false ); // low cone scoring position
      } else if ( 270 == iConPOV ) {
         // ArmToPosition( M_ARMPOS_BACKGRAB, false );  // Low back pickup
         ArmToPosition( M_ARMPOS_HUMNPLYRSTATN, false );  // Human player statn
        // m_OperatorController.SetRumble(
                            // frc::GenericHID::RumbleType::kBothRumble, 0.5 );
      } else {
         ArmByJoysticks();
      }  // if no "ArmToPosition()" button pressed
 
                                                  // open or close the grabber
      bBButton = m_OperatorController.GetBButton();
      if ( bBButton && !bBButton_prev ) {
         if ( bGrabberPortState || bGrabberStbdState ){
                                                        // open the grabber
            m_grabberPortSolenoid.Set( frc::DoubleSolenoid::Value::kReverse);
            m_grabberStbdSolenoid.Set( frc::DoubleSolenoid::Value::kReverse);
            bGrabberPortState = bGrabberStbdState = false; 
            sCurrState.sArmPose.bGrabberClosed = false;
         } else {
                                                        // close the grabber
            m_grabberPortSolenoid.Set( frc::DoubleSolenoid::Value::kForward);
            m_grabberStbdSolenoid.Set( frc::DoubleSolenoid::Value::kForward);
            bGrabberPortState = bGrabberStbdState = true;
            sCurrState.sArmPose.bGrabberClosed = true;
         }
      }

      bBButton_prev = bBButton;

      iCallCount++;

   }   // TeleopPeriodic()

};

#ifndef RUNNING_FRC_TESTS
int main()
{
   return frc::StartRobot<Robot>();
}
#endif
