// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "rev/CANSparkMax.h"
#include "rev/CANPIDController.h"
#include "rev/CANEncoder.h"
#include "ctre/Phoenix.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/encoder.h>
#include <frc/drive/DifferentialDrive.h>

using namespace frc;

  static const int m_leftLeadID = 3, m_rightLeadID = 1, m_leftFollowID = 4 , m_rightFollowID = 2;

  double rotations;
  double Distance;
  double pi = 3.1415926535;

  rev::CANSparkMax m_leftfront{m_leftLeadID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftback{m_leftFollowID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightfront{m_rightLeadID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightback{m_rightFollowID, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxPIDController m_pidController = m_leftfront.GetPIDController();
  rev::SparkMaxRelativeEncoder m_encoder = m_leftfront.GetEncoder();

  frc::DifferentialDrive m_drive{m_leftfront, m_rightfront};
  double kP = 0.85, kI = 0.00005, kD = 0.05, kIz = 0, kFF = 0, kMaxOutput = 0.2, kMinOutput = -0.2;


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  m_leftback.Follow(m_leftfront);
  m_rightback.Follow(m_rightfront);

  m_pidController.SetP(kP);
  m_pidController.SetI(kI);
  m_pidController.SetD(kD);
  m_pidController.SetIZone(kIz);
  m_pidController.SetFF(kFF);
  m_pidController.SetOutputRange(kMinOutput, kMaxOutput);
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
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
} 

void Move(double Distance) {
  rotations = (Distance/(6*pi))*8.68;
  m_pidController.SetReference(rotations, rev::ControlType::kPosition);
  }

void Robot::AutonomousPeriodic() { 
  if (m_autoSelected == kAutoNameCustom) {
    Move(20);
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
