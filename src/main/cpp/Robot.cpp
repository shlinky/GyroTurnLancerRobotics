/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  frontLeft = new Spark(frontLeftChannel);
  rearLeft = new Spark(rearLeftChannel);
  frontRight = new Spark(frontRightChannel);
  rearRight = new Spark(rearRightChannel);
  robotDrive = new MecanumDrive(*frontLeft, *rearLeft,
                                *frontRight, *rearRight);
  stick = new Joystick(joystickChannel);
	rotateToAngleRate = 0.0f;

  robotDrive->SetExpiration(0.1);
  frontLeft->SetInverted(true); // invert the left side motors
  rearLeft->SetInverted(true);  // (remove/modify to match your robot)
  try
  {
    /***********************************************************************
     * navX-MXP:
     * - Communication via RoboRIO MXP (SPI, I2C) and USB.            
     * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
     * 
     * navX-Micro:
     * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
     * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
     * 
     * VMX-pi:
     * - Communication via USB.
     * - See https://vmx-pi.kauailabs.com/installation/roborio-installation/
     * 
     * Multiple navX-model devices on a single robot are supported.
     ************************************************************************/
    ahrs = new AHRS(SPI::Port::kMXP);
  }
  catch (std::exception &ex)
  {
    std::string what_string = ex.what();
    std::string err_msg("Error instantiating navX MXP:  " + what_string);
    const char *p_err_msg = err_msg.c_str();
    DriverStation::ReportError(p_err_msg);
  }
  turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
  turnController->SetInputRange(-180.0f,  180.0f);
  turnController->SetOutputRange(-1.0, 1.0);
  turnController->SetAbsoluteTolerance(kToleranceDegrees);
  turnController->SetContinuous(true);
}

void Robot::PIDWrite(double output) {
  this->rotateToAngleRate = output;
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  bool reset_yaw_button_pressed = stick->GetRawButton(1);
  if ( reset_yaw_button_pressed ) {
      ahrs->ZeroYaw();
  }
  bool rotateToAngle = false;
  if ( stick->GetRawButton(2)) {
      turnController->SetSetpoint(0.0f);
      rotateToAngle = true;
  } else if ( stick->GetRawButton(3)) {
      turnController->SetSetpoint(90.0f);
      rotateToAngle = true;
  } else if ( stick->GetRawButton(4)) {
      turnController->SetSetpoint(179.9f);
      rotateToAngle = true;
  } else if ( stick->GetRawButton(5)) {
      turnController->SetSetpoint(-90.0f);
      rotateToAngle = true;
  }
  double currentRotationRate;
  if ( rotateToAngle ) {
      turnController->Enable();
      currentRotationRate = rotateToAngleRate;
  } else {
      turnController->Disable();
      currentRotationRate = stick->GetTwist();
  }
  try {
    /* Use the joystick X axis for lateral movement,          */
    /* Y axis for forward movement, and the current           */
    /* calculated rotation rate (or joystick Z axis),         */
    /* depending upon whether "rotate to angle" is active.    */
    robotDrive->DriveCartesian(stick->GetX(), stick->GetY(),
                              currentRotationRate, ahrs->GetAngle());
  } catch (std::exception& ex ) {
    std::string err_string = "Error communicating with Drive System:  ";
    err_string += ex.what();
    DriverStation::ReportError(err_string.c_str());
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif