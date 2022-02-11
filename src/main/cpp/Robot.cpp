#include "Robot.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "rev/CANSparkMax.h"
#include <frc/Joystick.h>
#include <frc/drive/differentialdrive.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"

using namespace frc;
using namespace std;

double leftleadmotorID = 1, rightleadmotorID = 3, leftfollowmotorID = 4 , rightfollowermotorID = 2;
  rev::CANSparkMax m_leftfront{leftleadmotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftback{leftfollowmotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightfront{rightleadmotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightback{rightfollowermotorID, rev::CANSparkMax::MotorType::kBrushless};

  frc::Joystick *m_stick;

  frc::DifferentialDrive drive{m_leftfront, m_rightfront};



void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  m_stick = new Joystick(1);

  m_leftback.Follow(m_leftfront);
  m_rightback.Follow(m_rightfront);

  //drives 18.85 inches per rotation; 8.68 motor revs * 42
  //365 rotations for 18.85 inches

}

void Robot::RobotPeriodic() {}

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

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  float KpX = -0.02;
  float KpY = -0.01;

  double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
  tx = (double) tx;
  double ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0);
  ty = (double) ty;

  if (m_stick->GetRawButton(2)){
    // nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("<tx>",0.0);
    // nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("<ty>",0.0);

    // float heading_error = tx;
    float steering_adjust = KpX * tx;
    float driving_adjust = KpY * ty;

    

    // if -ty then go forward
    // if + ty go backwards

    //float rotations = (steering_adjust/(6*pi))*8.68;

//     while (tx > 7.5 || tx < -7.5){
    //Rotation tracking
      drive.TankDrive(-steering_adjust, steering_adjust);
//     }
//     while (tx < 7.5 && tx > -7.5){
//       drive.TankDrive(driving_adjust, driving_adjust);
//     }


    //drive.TankDrive(-steering_adjust, steering_adjust);



    //drive.ArcadeDrive(0, -steering_adjust);

    // m_leftfront.Set(-steering_adjust);
    // m_rightfront.Set(steering_adjust);

    //drive.tankDrive(steering_adjust, -steering_adjust);

    //m_leftfront += steering_adjust;
    //m_rightfront -= steering_adjust;
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
