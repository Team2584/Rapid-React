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

   //THIS IS A PROBLEM #Fix this and the code should work as intended { 
  //std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    //2021 Network Table
  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable("limelight");
    //2022 Network Table //needs fix
  //  }

//   double tx = table->GetNumber("tx",0.0);                   //Get horizontal off set from target
//   double ty = table->GetNumber("ty",0.0);                   //Get vertical offset from target
//   double ta = table->GetNumber("ta",0.0);                   //Get area of target on screen
//   double ts = table->GetNumber("ts",0.0);                   //Get skew of target
//   double tv = table->GetNumber("tv", 0.0);

  nt::NetworkTableEntry tx;
  nt::NetworkTableEntry ty;
  nt::NetworkTableEntry ta;
  nt::NetworkTableEntry ts;
  nt::NetworkTableEntry tv;

  tx = table->GetEntry("tx");
  ty = table->GetEntry("ty");
  ta = table->GetEntry("ta");
  ts = table->GetEntry("ts");
  tv = table->GetEntry("tv");

  frc::Joystick *m_stick;

  frc::DifferentialDrive drive{m_leftfront, m_rightfront};

void Robot::RobotInit() {
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  m_stick = new Joystick(0);

  m_leftback.Follow(m_leftfront);
  m_rightback.Follow(m_rightfront);

  //drives 18.85 inches per rotation; 8.68 motor revs * 42
  //365 rotations for 18.85 inches
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // if (m_autonomousCommand != nullptr) {
  //   m_autonomousCommand->Cancel();
  //   m_autonomousCommand = nullptr;
  // }

  float KpX = -0.02;
  float KpY = -0.02;

  //THIS IS A PROBLEM #Fix this and the code should work as intended { 
  //std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    //2021 Network Table
  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable("limelight");
    //2022 Network Table //needs fix
  //  }

//   double tx = table->GetNumber("tx", 0.0);
//   double ty = table->GetNumber("ty", 0.0);  
  nt::NetworkTableEntry tx;
  nt::NetworkTableEntry ty;
  
  tx = table->GetEntry("tx");
  ty = table->GetEntry("ty");  

  if (m_stick->GetRawButton(2)){

    float heading_error = tx;
    float steering_adjust = KpX * tx;
    float driving_adjust = KpY * ty;

    // if -ty then go forward
    // if + ty go backwards

    //float rotations = (steering_adjust/(6*pi))*8.68;

    while (tx > 7.5 || tx < -7.5){
      drive.TankDrive(-steering_adjust, steering_adjust);
    }
    while (tx < 7.5 && tx > -7.5){
      drive.TankDrive(driving_adjust, driving_adjust);
    }


    //drive.TankDrive(-steering_adjust, steering_adjust);



    //drive.ArcadeDrive(0, -steering_adjust);

    // m_leftfront.Set(-steering_adjust);
    // m_rightfront.Set(steering_adjust);

    //drive.tankDrive(steering_adjust, -steering_adjust);

    //m_leftfront += steering_adjust;
    //m_rightfront -= steering_adjust;
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
