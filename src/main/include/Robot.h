/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <AHRS.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/buttons/JoystickButton.h>
#include <rev/CANSparkMax.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  const static double kP = 0.03f;
  const static double kI = 0.00f;
  const static double kD = 0.00f;
  const static double kF = 0.00f;

  /* This tuning parameter indicates how close to "on target" the    */
  /* PID Controller will attempt to get.                             */
  Spark* frontLeft; 
  Spark* rearLeft; 
  Spark* frontRight; 
  Spark* rearRight;
  MecanumDrive* robotDrive;
  JoyStick* stick;

  const static double kToleranceDegrees = 2.0f;
  AHRS* ahrs;
};
