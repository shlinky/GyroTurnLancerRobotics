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
#include <frc/Joystick.h>
#include <rev/CANSparkMax.h>
#include <frc/buttons/JoystickButton.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/RobotDrive.h>
#include <frc/DriverStation.h>
#include <frc/PIDOutput.h>
#include <frc/PIDController.h>

using namespace frc;
class Robot : public frc::TimedRobot,
              public frc::PIDOutput {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void PIDWrite(double output);
 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  constexpr static double kP = 0.03f;
  constexpr static double kI = 0.00f;
  constexpr static double kD = 0.00f;
  constexpr static double kF = 0.00f;

  /* This tuning parameter indicates how close to "on target" the    */
  /* PID Controller will attempt to get.                             */

  static const int lfID = 1, lbID = 2, rfID = 3, rbID = 4;

  rev::CANSparkMax* frontLeft = new rev::CANSparkMax(lfID, rev::CANSparkMax::MotorType::kBrushless);
  //rev::CANSparkMax* rearLeft = new rev::CANSparkMax(lbID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* frontRight = new rev::CANSparkMax(rfID, rev::CANSparkMax::MotorType::kBrushless);
  //rev::CANSparkMax* rearRight = new rev::CANSparkMax(rbID, rev::CANSparkMax::MotorType::kBrushless);

  RobotDrive* robotDrive;
  Joystick* stick;
  PIDController* turnController;

  float rotateToAngleRate;
  constexpr static double kToleranceDegrees = 2.0f;
  AHRS* ahrs;
};
