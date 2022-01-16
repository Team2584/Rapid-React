#include <Robot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "rev/CANSparkMax.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <frc/Joystick.h>
#include <frc/drive/differentialdrive.h>

using namespace frc;

  double leftleadmotorID = 3, rightleadmotorID = 1, leftfollowmotorID = 4 , rightfollowermotorID = 2;
  rev::CANSparkMax m_leftfront{leftleadmotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftback{leftfollowmotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightfront{rightleadmotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightback{rightfollowermotorID, rev::CANSparkMax::MotorType::kBrushless};

  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double tx = table->GetNumber("tx",0.0);                   //Get horizontal off set from target
  double ty = table->GetNumber("ty",0.0);                   //Get vertical offset from target
  double ta = table->GetNumber("ta",0.0);                   //Get area of target on screen
  double ts = table->GetNumber("ts",0.0);                   //Get skew of target
  double tv = table->GetNumber("tv", 0.0);

  rev::CANEncoder m_encoderR = m_rightfront.GetEncoder();
  rev::CANEncoder m_encoderL = m_leftfront.GetEncoder();

  frc::Joystick *m_stick;

  frc::DifferentialDrive drive{m_leftfront, m_rightfront};

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  m_stick = new Joystick(0);

  m_leftback.Follow(m_leftfront);
  m_rightback.Follow(m_rightfront);

  m_encoderR.SetPosition(0.0);
  m_encoderL.SetPosition(0.0);

  //drives 18.85 inches per rotation; 8.68 motor revs * 42
  //365 rotations for 18.85 inches

}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  /*
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  // fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
  */
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

  float Kp = -0.02;

  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double tx = table->GetNumber("tx", 0.0);  

  if (m_stick->GetRawButton(2)){

    float heading_error = tx;
    float steering_adjust = Kp * tx;

    //float rotations = (steering_adjust/(6*pi))*8.68;

    drive.TankDrive(-steering_adjust, steering_adjust);
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
