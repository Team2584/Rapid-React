// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"

#include <stdlib.h>
#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/Joystick.h>

using namespace frc;

static const int m_passiveIntakeID = 1, m_towerID = 2;

rev::CANSparkMax m_passiveIntakeMotor{m_passiveIntakeID, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax m_towerMotor{m_towerID, rev::CANSparkMax::MotorType::kBrushed};

frc::Joystick *cont_Driver;
frc::Joystick *cont_Partner;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);

  cont_Driver = new Joystick(0);
  cont_Partner = new Joystick(1);

  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  //passive intake side set to Driver Contoller RTrigger between 0 and 75% power linear currently not inverted
  m_passiveIntakeMotor.Set(cont_Driver->GetRawAxis(4)*0.75);

  //tower ball lift set to Partner Controller X const 50% power currently not inverted
  if (cont_Partner->GetRawButton(2)){
    m_towerMotor.Set(0.5);
  }

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
