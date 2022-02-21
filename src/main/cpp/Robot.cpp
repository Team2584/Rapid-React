// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "ctre/Phoenix.h"
#include <rev/CANEncoder.h>
#include <rev/CANSparkMax.h>
#include <rev/ColorSensorV3.h>
//#include "rev/CANEncoder.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/PS4Controller.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/fmt/Units.h>

#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/controller/PIDController.h>
#include <frc/DigitalInput.h>
#include <frc/encoder.h>

#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "cameraserver/CameraServer.h"

#include <iostream>
#include <thread>
#include <math.h>

using namespace std;
using namespace frc;
using namespace frc2;
using namespace rev;

//General 
Compressor _compressor(frc::PneumaticsModuleType::CTREPCM);

//rev::ColorSensorV3 col_frontleftColorSensor{frc::I2C::Port::kOnboard};
int autonState = 0;

auto inst = nt::NetworkTableInstance::GetDefault();
auto limeTable = inst.GetTable("limelight");

frc2::PIDController autoAimPID(0.05, 0.001, .005);
double tx = limeTable->GetNumber("tx",0.0);                   //Get horizontal off set from target
double ty = limeTable->GetNumber("ty",0.0);                   //Get vertical offset from target
double ta = limeTable->GetNumber("ta",0.0);                   //Get area of target on screen

//Drive Base
static const int m_leftLeadID = 1, m_rightLeadID = 3, m_leftFollowID = 2, m_rightFollowID = 4;
rev::CANSparkMax m_leftLeadMotor{m_leftLeadID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_rightLeadMotor{m_rightLeadID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_leftFollowMotor{m_leftFollowID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_rightFollowMotor{m_rightFollowID, rev::CANSparkMax::MotorType::kBrushless};
double m_leftOut = 0, m_rightOut = 0;

double drivekP = 6e-5, drivekI = 1e-6, drivekD = 0, drivekIz = 0, drivekFF = 0.000015, drivekMaxOutput = 1.0, drivekMinOutput = -1.0, driveMaxRPM = 5108, driveGearRatio = 8.68;
rev::SparkMaxPIDController m_leftPID = m_leftLeadMotor.GetPIDController();
rev::SparkMaxPIDController m_rightPID = m_rightLeadMotor.GetPIDController();

rev::SparkMaxRelativeEncoder m_leftEncoder = m_leftLeadMotor.GetEncoder();
rev::SparkMaxRelativeEncoder m_rightEncoder = m_rightLeadMotor.GetEncoder();

frc::DifferentialDrive m_drive{m_leftLeadMotor, m_rightLeadMotor};
frc2::PIDController driveControl{drivekP, drivekI, drivekD};

WPI_PigeonIMU _gyro(1);
Rotation2d getGyroAngle(){
  double gyroroation = _gyro.GetFusedHeading();
  return Rotation2d(units::degree_t(gyroroation));
}

//Flywheel
WPI_TalonFX m_flywheelMotor{12};
frc2::PIDController m_flywheelPID{0.0008, 0, 0, units::time::second_t(50)};
//frc2::PIDController m_flywheelPID{}
double flywheelGearRatio = 1;
double flywheelMaxRPM = 5742/*max rpm of falcon 500 no load -10%*/ * flywheelGearRatio, flywheelMinRPM = 5742 * -0.1/*percent output limit for reverse*/ * flywheelGearRatio;
double flywheelTargetRPM = 0;

//Path Weaver
double ks, kv, ka;
Pose2d position(units::meter_t(0), units::meter_t(0), Rotation2d(units::degree_t(135)));
DifferentialDriveKinematics kinematics(units::length::meter_t(0.7633));
//DifferentialDriveOdometry odometry(getGyroAngle(), position);
frc::SimpleMotorFeedforward<units::meters> feedfwd(0.22_V, 1.98 * 1_V * 1_s / 1_m, 0.2 * 1_V * 1_s * 1_s / 1_m);

//Intake, Ball runs
static const int m_intakeFrontID = 9, m_intakeBackID = 7, m_lowerTowerID = 6, m_indexerID = 5;
rev::CANSparkMax m_intakeFrontMotor{m_intakeFrontID, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax m_intakeBackMotor{m_intakeBackID, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax m_lowerTowerMotor{m_lowerTowerID, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax m_indexerMotor{m_indexerID, rev::CANSparkMax::MotorType::kBrushed};
DoubleSolenoid sol_frontIntakeSolenoid(frc::PneumaticsModuleType::CTREPCM, 2, 3);

//Lift and Climb
static const int m_leftWhinchID = 11, m_rightWhinchID = 10;
rev::CANSparkMax m_leftWhinchMotor{m_leftWhinchID, rev::CANSparkMax::MotorType::kBrushless};
DoubleSolenoid sol_climber(frc::PneumaticsModuleType::CTREPCM, 0, 1);
rev::CANSparkMax m_rightWhinchMotor{m_rightWhinchID, rev::CANSparkMax::MotorType::kBrushless};

//Controllers
PS4Controller *cont_Driver = new PS4Controller(0);
PS4Controller *cont_Partner = new PS4Controller(1);
double joy_lStick_Y_deadband = 0.05, joy_rStick_Y_deadband = 0.05, joy_rStick_X_deadband = 0.05;

double getLeftEncoderDist(){
  return m_leftEncoder.GetPosition() / 42/*ticks per rev*/ * driveGearRatio * (2 * M_PI * 0.0762)/*dist per rev in meters*/;
}

double getRightEncoderDist(){
  return m_rightEncoder.GetPosition() / 42 * driveGearRatio * (2 * M_PI * 0.0762);
}

void move(double dist){
  double rotations = (dist/(6*M_PI))*8.68;
  //DifferentialDrive::
}

void Robot::RobotInit() {
  cs::UsbCamera limelight = CameraServer::GetInstance()->StartAutomaticCapture(0);
  limelight.SetResolution(200, 150);
  limelight.SetFPS(40);
  _compressor.EnableDigital();
  
  m_leftFollowMotor.Follow(m_leftLeadMotor, false);
  //m_leftLeadMotor.SetInverted(true);
  m_rightFollowMotor.Follow(m_rightLeadMotor, false);
  //m_rightLeadMotor.SetInverted(true);

  m_leftLeadMotor.SetInverted(true);

  m_leftLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_leftFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_rightLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_rightFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

  m_leftPID.SetP(drivekP);
  m_leftPID.SetI(drivekI);
  m_leftPID.SetD(drivekD);
  m_leftPID.SetIZone(drivekIz);
  m_leftPID.SetFF(drivekFF);
  m_leftPID.SetOutputRange(drivekMinOutput, drivekMaxOutput);

  m_rightPID.SetP(drivekP);
  m_rightPID.SetI(drivekI);
  m_rightPID.SetD(drivekD);
  m_rightPID.SetIZone(drivekIz);
  m_rightPID.SetFF(drivekFF);
  m_rightPID.SetOutputRange(drivekMinOutput, drivekMaxOutput);

  m_rightEncoder.SetPosition(0.0);
  m_leftEncoder.SetPosition(0.0);

  m_intakeBackMotor.SetInverted(true);
  m_intakeFrontMotor.SetInverted(true);
  m_indexerMotor.SetInverted(true);

  m_rightWhinchMotor.Follow(m_leftWhinchMotor);
  m_leftWhinchMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightWhinchMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  _gyro.SetFusedHeading(0);

  SmartDashboard::PutNumber("Min Belt Percent", 0.5);
  SmartDashboard::PutNumber("Min Intake Percent", 0.5);
  SmartDashboard::PutNumber("Min Indexer Percent", 0.5);
  SmartDashboard::PutNumber("Flywheel Far RPM", 5742 * flywheelGearRatio * 0.8);
  SmartDashboard::PutNumber("Flywheel Tarmac RPM", 5742 * flywheelGearRatio * 0.6);
  SmartDashboard::PutNumber("Flywheel Fender RPM", 5742 * flywheelGearRatio * 0.4);
  SmartDashboard::PutNumber("Whinch Percent", 0.5);
}

void Robot::RobotPeriodic() {
  //position = odometry->Update(getGyroAngle(), units::meter_t(getLeftEncoderDist()), units::meter_t(getRightEncoderDist()));
  autoAimPID.SetSetpoint(0);
}

void Robot::AutonomousInit() {
  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
  autonState = 0;
  _gyro.SetFusedHeading(0);
  m_leftLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_leftFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Robot::AutonomousPeriodic() {
  /*switch(autonState){
    case 0:
      m_intakeBackMotor.Set(0.5);
      autonState += 1;
      break;
    case 1:
      /*m_drive.ArcadeDrive(-0.5, 0);
      if(m_leftEncoder.GetPosition() > 10/(-24*M_PI)*8.68){
        autonState += 1;
      }*//*
      autonState += 1;
      break;
    case 2:
      m_flywheelMotor.Set(0.6);
      if (m_flywheelMotor.GetSelectedSensorVelocity() / 2048/*Units per rotation*/// * 10/*100ms to 1000ms/1s*/ * 60/*1s to 60s/1m*/ * flywheelGearRatio > flywheelMaxRPM*0.55){
        /*autonState += 1;
      }
      break;
    case 3:
      m_indexerMotor.Set(0.5);
      m_lowerTowerMotor.Set(0.5);
      break;

  }*/
  /*
  m_leftPID.SetReference((10/(6*M_PI))*8.68, rev::ControlType::kPosition);
  if (m_leftPID.SetReference((10/(6*M_PI))*8.68, rev::ControlType::kPosition) == REVLibError::kOk){
    m_intakeFrontMotor.Set(0.5);
  }*/
  /*switch(autonState){
    case 0:
      m_drive.ArcadeDrive(0, 0.5);
      if (_gyro.GetFusedHeading() >= 90*0.9){
        m_drive.ArcadeDrive(0,0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        autonState += 1;
      }
      break;
  }*/
  switch(autonState){
    case 0:
      //m_intakeBackMotor.Set(0.5);
      m_drive.ArcadeDrive(0.5, 0);
      SmartDashboard::PutNumber("Encoder", m_leftEncoder.GetPosition());
      if (m_leftEncoder.GetPosition() >= ((40/(6*M_PI))*8.68)){
        autonState += 1;
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        _gyro.SetFusedHeading(0);
        Wait(units::time::second_t(1));
      }
      break;
    case 1:
      m_flywheelMotor.Set(0.6);
      m_drive.ArcadeDrive(-0.5, 0);
      if (m_leftEncoder.GetPosition() <= ((-40/(6*M_PI))*8.68)){
        autonState += 1;
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        _gyro.SetFusedHeading(0);
      }
      break;
    case 2:
      if (m_flywheelMotor.GetSelectedSensorVelocity() * 600 / 2048 >= flywheelMaxRPM * 0.55){
        m_lowerTowerMotor.Set(0.6);
        m_indexerMotor.Set(0.6);
        Wait(units::time::second_t(2.0));
        autonState += 1;
        m_flywheelMotor.Set(0);
        m_indexerMotor.Set(0);
        m_lowerTowerMotor.Set(0);
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        _gyro.SetFusedHeading(0);
      }
      break;
    case 3:
      m_drive.ArcadeDrive(0, -0.5);
      if (_gyro.GetFusedHeading() <= -45){
        autonState += 1;
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        _gyro.SetFusedHeading(0);
      }
      break;
    case 4:
      m_drive.ArcadeDrive(0.5, 0);
      if (m_leftEncoder.GetPosition() >= ((70/(6*M_PI))*8.68)){
        autonState += 1;
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        _gyro.SetFusedHeading(0);
      }
      break;
    case 5:
      //start go to furthest back ball
      //run belts for a second or so
      //remember to not immediately switch directions
      break;
    case 6:
      //start flywheel
      //go forward to bumper
      break;
    case 7:
      //shoot balls
    
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  SmartDashboard::PutNumber("Encoder", m_leftEncoder.GetPosition());
  /*
   .----------------.  .----------------.  .----------------.  .----------------.  .----------------.  .----------------.  .----------------. 
  | .--------------. || .--------------. || .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |
  | |     ______   | || |   _____      | || |     _____    | || | ____    ____ | || |   ______     | || |  _________   | || |  _______     | |
  | |   .' ___  |  | || |  |_   _|     | || |    |_   _|   | || ||_   \  /   _|| || |  |_   _ \    | || | |_   ___  |  | || | |_   __ \    | |
  | |  / .'   \_|  | || |    | |       | || |      | |     | || |  |   \/   |  | || |    | |_) |   | || |   | |_  \_|  | || |   | |__) |   | |
  | |  | |         | || |    | |   _   | || |      | |     | || |  | |\  /| |  | || |    |  __'.   | || |   |  _|  _   | || |   |  __ /    | |
  | |  \ `.___.'\  | || |   _| |__/ |  | || |     _| |_    | || | _| |_\/_| |_ | || |   _| |__) |  | || |  _| |___/ |  | || |  _| |  \ \_  | |
  | |   `._____.'  | || |  |________|  | || |    |_____|   | || ||_____||_____|| || |  |_______/   | || | |_________|  | || | |____| |___| | |
  | |              | || |              | || |              | || |              | || |              | || |              | || |              | |
  | '--------------' || '--------------' || '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |
   '----------------'  '----------------'  '----------------'  '----------------'  '----------------'  '----------------'  '----------------' 
  */
  if (cont_Driver->GetPOV() == 90){
    sol_climber.Set(frc::DoubleSolenoid::Value::kForward);
    //sol_rightWhinchSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
  }
  else if (cont_Driver->GetPOV() == 270){
    sol_climber.Set(frc::DoubleSolenoid::Value::kReverse);
    //sol_rightWhinchSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
  }

  if (cont_Driver->GetTriangleButton()){
    m_leftWhinchMotor.Set(0.5);
    m_rightWhinchMotor.Set(0.5);
  }
  else if (cont_Driver->GetCrossButton()){
    m_leftWhinchMotor.Set(-0.5);
    m_rightWhinchMotor.Set(-0.5);
  }
  else {
    m_leftWhinchMotor.Disable();
    m_rightWhinchMotor.Disable();
  }

  /*
  .----------------.  .-----------------. .----------------.  .----------------.  .----------------.  .----------------.  .----------------. 
  | .--------------. || .--------------. || .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |
  | |     _____    | || | ____  _____  | || |  ________    | || |  _________   | || |  ____  ____  | || |  _________   | || |  _______     | |
  | |    |_   _|   | || ||_   \|_   _| | || | |_   ___ `.  | || | |_   ___  |  | || | |_  _||_  _| | || | |_   ___  |  | || | |_   __ \    | |
  | |      | |     | || |  |   \ | |   | || |   | |   `. \ | || |   | |_  \_|  | || |   \ \  / /   | || |   | |_  \_|  | || |   | |__) |   | |
  | |      | |     | || |  | |\ \| |   | || |   | |    | | | || |   |  _|  _   | || |    > `' <    | || |   |  _|  _   | || |   |  __ /    | |
  | |     _| |_    | || | _| |_\   |_  | || |  _| |___.' / | || |  _| |___/ |  | || |  _/ /'`\ \_  | || |  _| |___/ |  | || |  _| |  \ \_  | |
  | |    |_____|   | || ||_____|\____| | || | |________.'  | || | |_________|  | || | |____||____| | || | |_________|  | || | |____| |___| | |
  | |              | || |              | || |              | || |              | || |              | || |              | || |              | |
  | '--------------' || '--------------' || '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |
   '----------------'  '----------------'  '----------------'  '----------------'  '----------------'  '----------------'  '----------------' 
  */
  if (cont_Partner->GetR2Axis() >= SmartDashboard::GetNumber("Min Indexer Percent", 0.5)){
    m_indexerMotor.Set(cont_Driver->GetR2Axis());
  }
  else if (cont_Partner->GetR2Button()){
    m_indexerMotor.Set(SmartDashboard::GetNumber("Min Indexer Percent", 0.5));
  }
  else if (cont_Partner->GetR1Button()){
    m_indexerMotor.Set(0.2);
  }
  else {
    m_indexerMotor.Set(0);
  }

  /*
  .----------------.  .----------------.  .----------------.  .----------------.  .----------------. 
  | .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |
  | |  _________   | || |     ____     | || | _____  _____ | || |  _________   | || |  _______     | |
  | | |  _   _  |  | || |   .'    `.   | || ||_   _||_   _|| || | |_   ___  |  | || | |_   __ \    | |
  | | |_/ | | \_|  | || |  /  .--.  \  | || |  | | /\ | |  | || |   | |_  \_|  | || |   | |__) |   | |
  | |     | |      | || |  | |    | |  | || |  | |/  \| |  | || |   |  _|  _   | || |   |  __ /    | |
  | |    _| |_     | || |  \  `--'  /  | || |  |   /\   |  | || |  _| |___/ |  | || |  _| |  \ \_  | |
  | |   |_____|    | || |   `.____.'   | || |  |__/  \__|  | || | |_________|  | || | |____| |___| | |
  | |              | || |              | || |              | || |              | || |              | |
  | '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |
   '----------------'  '----------------'  '----------------'  '----------------'  '----------------'
  */
  if (cont_Driver->GetR2Axis() >= SmartDashboard::GetNumber("Min Belt Percent", 0.5) || cont_Partner->GetL2Axis() >= SmartDashboard::GetNumber("Min Belt Percent", 0.5)){
    m_lowerTowerMotor.Set((cont_Driver->GetR2Axis() > cont_Partner->GetL2Axis()) ? cont_Driver->GetR2Axis() : cont_Partner->GetL2Axis());
  }
  else if (cont_Driver->GetR2Button() || cont_Partner->GetL2Button()){
    m_lowerTowerMotor.Set(SmartDashboard::GetNumber("Min Belt Percent", 0.5));
  }
  else if (cont_Driver->GetR1Button() || cont_Partner->GetL1Button()){
    m_lowerTowerMotor.Set(-0.2);
  }
  else {
    m_lowerTowerMotor.Set(0);
  }

  /*
   .----------------.  .-----------------. .----------------.  .----------------.  .----------------.  .----------------. 
  | .--------------. || .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |
  | |     _____    | || | ____  _____  | || |  _________   | || |      __      | || |  ___  ____   | || |  _________   | |
  | |    |_   _|   | || ||_   \|_   _| | || | |  _   _  |  | || |     /  \     | || | |_  ||_  _|  | || | |_   ___  |  | |
  | |      | |     | || |  |   \ | |   | || | |_/ | | \_|  | || |    / /\ \    | || |   | |_/ /    | || |   | |_  \_|  | |
  | |      | |     | || |  | |\ \| |   | || |     | |      | || |   / ____ \   | || |   |  __'.    | || |   |  _|  _   | |
  | |     _| |_    | || | _| |_\   |_  | || |    _| |_     | || | _/ /    \ \_ | || |  _| |  \ \_  | || |  _| |___/ |  | |
  | |    |_____|   | || ||_____|\____| | || |   |_____|    | || ||____|  |____|| || | |____||____| | || | |_________|  | |
  | |              | || |              | || |              | || |              | || |              | || |              | |
  | '--------------' || '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |
   '----------------'  '----------------'  '----------------'  '----------------'  '----------------'  '----------------'
  */
  if (cont_Driver->GetL2Axis() >= SmartDashboard::GetNumber("Min Intake Percent", 0.5)){
    m_intakeFrontMotor.Set(cont_Driver->GetL2Axis());
    m_intakeBackMotor.Set(cont_Driver->GetL2Axis());
  }
  else if (cont_Driver->GetL2Button()){
    m_intakeFrontMotor.Set(SmartDashboard::GetNumber("Min Intake Percent", 0.5));
    m_intakeBackMotor.Set(SmartDashboard::GetNumber("Min Intake Percent", 0.5));
  }
  else if (cont_Driver->GetL1Button()){
    m_intakeBackMotor.Set(-0.2);
    m_intakeFrontMotor.Set(-0.2);
  }
  else {
    m_intakeBackMotor.Set(0);
    m_intakeFrontMotor.Set(0);
  }

  /*if (cont_Driver->GetPOV() == 0, 315, 45){
    sol_frontIntakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
  }
  else if (cont_Driver->GetPOV() == 180, 225, 135){
    sol_frontIntakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
  }*/

  /*
  .----------------.  .----------------.  .----------------.  .----------------.  .----------------.  .----------------.  .----------------.  .----------------. 
  | .--------------. || .--------------. || .--------------. || .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |
  | |  _________   | || |   _____      | || |  ____  ____  | || | _____  _____ | || |  ____  ____  | || |  _________   | || |  _________   | || |   _____      | |
  | | |_   ___  |  | || |  |_   _|     | || | |_  _||_  _| | || ||_   _||_   _|| || | |_   ||   _| | || | |_   ___  |  | || | |_   ___  |  | || |  |_   _|     | |
  | |   | |_  \_|  | || |    | |       | || |   \ \  / /   | || |  | | /\ | |  | || |   | |__| |   | || |   | |_  \_|  | || |   | |_  \_|  | || |    | |       | |
  | |   |  _|      | || |    | |   _   | || |    \ \/ /    | || |  | |/  \| |  | || |   |  __  |   | || |   |  _|  _   | || |   |  _|  _   | || |    | |   _   | |
  | |  _| |_       | || |   _| |__/ |  | || |    _|  |_    | || |  |   /\   |  | || |  _| |  | |_  | || |  _| |___/ |  | || |  _| |___/ |  | || |   _| |__/ |  | |
  | | |_____|      | || |  |________|  | || |   |______|   | || |  |__/  \__|  | || | |____||____| | || | |_________|  | || | |_________|  | || |  |________|  | |
  | |              | || |              | || |              | || |              | || |              | || |              | || |              | || |              | |
  | '--------------' || '--------------' || '--------------' || '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |
   '----------------'  '----------------'  '----------------'  '----------------'  '----------------'  '----------------'  '----------------'  '----------------' 
  */
  double flywheelRPM = m_flywheelMotor.GetSelectedSensorVelocity() / 2048/*Units per rotation*/ * 10/*100ms to 1000ms/1s*/ * 60/*1s to 60s/1m*/ * flywheelGearRatio;
  if (cont_Partner->GetTriangleButtonPressed()){
    flywheelTargetRPM = SmartDashboard::GetNumber("Flywheel Far RPM", 5742 * flywheelGearRatio * 0.8); 
    m_flywheelPID.SetSetpoint(flywheelTargetRPM);
  }
  else if (cont_Partner->GetSquareButtonPressed()){
    flywheelTargetRPM = SmartDashboard::GetNumber("Flywheel Tarmac RPM", 5742 * flywheelGearRatio * 0.6);
    m_flywheelPID.SetSetpoint(flywheelTargetRPM);
  }
  else if (cont_Partner->GetCircleButtonPressed()){
    flywheelTargetRPM = SmartDashboard::GetNumber("Flywheel Fender RPM", 5742 * flywheelGearRatio * 0.4);
    m_flywheelPID.SetSetpoint(flywheelTargetRPM);
  }
  else if (cont_Partner->GetCrossButtonPressed()){
    flywheelTargetRPM = 0;
    m_flywheelPID.SetSetpoint(flywheelTargetRPM);
  }
  SmartDashboard::PutNumber("Flywheel RPM", flywheelRPM);
  SmartDashboard::PutNumber("Flywheel Target RPM", flywheelTargetRPM);
  double output = std::clamp(m_flywheelPID.Calculate(flywheelRPM), flywheelMinRPM, flywheelMaxRPM);
  SmartDashboard::PutNumber("Flywheel Output", output);
  m_flywheelMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output);

  /*
  .----------------.  .----------------.  .----------------.  .----------------.  .----------------. 
  | .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |
  | |  ________    | || |  _______     | || |     _____    | || | ____   ____  | || |  _________   | |
  | | |_   ___ `.  | || | |_   __ \    | || |    |_   _|   | || ||_  _| |_  _| | || | |_   ___  |  | |
  | |   | |   `. \ | || |   | |__) |   | || |      | |     | || |  \ \   / /   | || |   | |_  \_|  | |
  | |   | |    | | | || |   |  __ /    | || |      | |     | || |   \ \ / /    | || |   |  _|  _   | |
  | |  _| |___.' / | || |  _| |  \ \_  | || |     _| |_    | || |    \ ' /     | || |  _| |___/ |  | |
  | | |________.'  | || | |____| |___| | || |    |_____|   | || |     \_/      | || | |_________|  | |
  | |              | || |              | || |              | || |              | || |              | |
  | '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |
   '----------------'  '----------------'  '----------------'  '----------------'  '----------------' 
  */
  double joy_lStick_Y = cont_Driver->GetLeftY(), joy_rStick_Y = cont_Driver->GetRightY(), joy_rStick_X = cont_Driver->GetRightX();

  if (abs(joy_lStick_Y) < joy_lStick_Y_deadband){
    joy_lStick_Y = 0;
  }
  else{
    //joy_lStick_Y;
  }

  if (abs(joy_rStick_Y) < joy_rStick_Y_deadband){
    joy_rStick_Y = 0;
  }
  else{
    //joy_rStick_Y;
  }

  if (abs(joy_rStick_X) < joy_rStick_X_deadband){
    joy_rStick_X = 0;
  }
  else{
    //joy_rStick_X;
  }

  if (cont_Partner->GetTouchpad()){
    double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
    double steeringAdj = autoAimPID.Calculate(tx >= 0 ? std::clamp(tx, 0.5, 10.0) : std::clamp(tx, -10.0, -0.5));
    m_drive.ArcadeDrive(joy_lStick_Y, steeringAdj);
  }
  else{
    m_drive.ArcadeDrive(joy_lStick_Y*0.8, -joy_rStick_X*0.8);
  }

  //Arcade
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
