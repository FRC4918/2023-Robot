// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#define SAFETY_LIMITS 1

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <iostream>

#include "frc/PneumaticsModuleType.h"
#include "frc/Compressor.h"
#include "frc/DigitalInput.h"
#include "frc/DigitalOutput.h"
#include "frc/DigitalSource.h"
#include "frc/DoubleSolenoid.h"

#include "Drivetrain.h"

                                                            // CTRE compressor
   frc::Compressor m_compressor{ 0, frc::PneumaticsModuleType::CTREPCM };

   frc::DoubleSolenoid m_grabberPortSolenoid{
                                  0, frc::PneumaticsModuleType::CTREPCM, 4, 6};
   frc::DoubleSolenoid m_grabberStbdSolenoid{
                                  0, frc::PneumaticsModuleType::CTREPCM, 5, 7};

class Robot : public frc::TimedRobot
{

#define JAG_TEMPSHOULDERMOTORTEST = 1
#ifdef JAG_TEMPSHOULDERMOTORTEST
   void MotorInitSpark(rev::CANSparkMax &m_motor);
   static const int ShoulderMotorDeviceID =  16;
   rev::CANSparkMax m_ShoulderMotor{ ShoulderMotorDeviceID,
                                     rev::CANSparkMax::MotorType::kBrushless};
#endif

public:
   void RobotInit() override
   {
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
      DriveWithJoystick(false);
      m_swerve.UpdateOdometry();
   }

#ifdef JAG_TEMPSHOULDERMOTORTEST
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

   void TeleopInit() override
   {
      ArmMotorInitSpark( m_ShoulderMotor );
      m_compressor.EnableDigital();
   }
#endif

   // change to true for field relative
   //          false for robot relative
   void TeleopPeriodic() override
   {
      static bool bGrabberPortState = false;       // Is portside  grabber closed?
      static bool bGrabberStbdState = false;       // Is starboard grabber closed?

      static bool bBButton = false;
      static bool bBButton_prev = false;


      DriveWithJoystick(true);
#ifdef JAG_TEMPSHOULDERMOTORTEST
                                    // Run the shoulder motor to rotate the arm
      m_ShoulderMotor.SetVoltage(
        units::volt_t{
#ifdef SAFETY_LIMITS
	             // Be very careful with this motor; it is geared down so
		     // low that it could easily break things if left here
		     // at a high amp limit (high torque limit)
        -frc::ApplyDeadband(m_OperatorController.GetRightY(), 0.10) } *  1.0 );
#else
        -frc::ApplyDeadband(m_OperatorController.GetRightY(), 0.10) } * 12.0 );
#endif
#endif

                                                  // open or close the grabber
      bBButton = m_controller.GetBButton();
      if ( bBButton != bBButton_prev ) {
         if ( bGrabberPortState || bGrabberStbdState ){
                                                           // close the grabber
            m_grabberPortSolenoid.Set( frc::DoubleSolenoid::Value::kForward);
            m_grabberStbdSolenoid.Set( frc::DoubleSolenoid::Value::kForward);
            bGrabberPortState = bGrabberStbdState = true; 
         } else {
                                                           // open the grabber
            m_grabberPortSolenoid.Set( frc::DoubleSolenoid::Value::kReverse);
            m_grabberStbdSolenoid.Set( frc::DoubleSolenoid::Value::kReverse);
            bGrabberPortState = bGrabberStbdState = false; 
         }
      }
      bBButton_prev = bBButton;

   }

private:
   frc::XboxController m_controller{0};
   frc::XboxController m_OperatorController{1};
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
   }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
   return frc::StartRobot<Robot>();
}
#endif
