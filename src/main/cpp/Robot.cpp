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
#include <frc/DigitalInput.h>

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

cs::UsbCamera backCam;
cs::UsbCamera frontCam;
nt::NetworkTableEntry cameraSelection;

//General 
Compressor _compressor(frc::PneumaticsModuleType::CTREPCM);

int autonState = 0;

auto inst = nt::NetworkTableInstance::GetDefault();
auto limeTable = inst.GetTable("limelight");

frc2::PIDController autoAimPID(0.05, 0.001, .005);

//Drive Base
static const int m_leftLeadID = 1, m_rightLeadID = 3, m_leftFollowID = 2, m_rightFollowID = 4;
rev::CANSparkMax m_leftLeadMotor{m_leftLeadID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_rightLeadMotor{m_rightLeadID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_leftFollowMotor{m_leftFollowID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_rightFollowMotor{m_rightFollowID, rev::CANSparkMax::MotorType::kBrushless};
double m_leftOut = 0, m_rightOut = 0;
double driveGearRatio = 8.68;

rev::SparkMaxRelativeEncoder m_leftEncoder = m_leftLeadMotor.GetEncoder();
rev::SparkMaxRelativeEncoder m_rightEncoder = m_rightLeadMotor.GetEncoder();

frc::DifferentialDrive m_drive{m_leftLeadMotor, m_rightLeadMotor};

WPI_PigeonIMU _gyro(0);
Rotation2d getGyroAngle(){
  double gyroroation = _gyro.GetFusedHeading();
  return Rotation2d(units::degree_t(gyroroation));
}

//Flywheel
WPI_TalonFX m_flywheelMotor{12};
frc2::PIDController m_flywheelPID{0.0008, 0, 0, units::time::second_t(50)};
double flywheelGearRatio = 1;
double flywheelMaxRPM = 5742/*max rpm of falcon 500 no load -10%*/ * flywheelGearRatio, flywheelMinRPM = 5742 * -0.1/*percent output limit for reverse*/ * flywheelGearRatio;
double flywheelTargetRPM = 0;
double flywheelTargetPCT = 0;
bool shot1Taken, shot2Taken = false;

//Path Weaver
double ks, kv, ka;
Pose2d position(units::meter_t(0), units::meter_t(0), Rotation2d(units::degree_t(135)));
DifferentialDriveKinematics kinematics(units::length::meter_t(0.7633));
DifferentialDriveOdometry odometry(getGyroAngle(), position);
frc::SimpleMotorFeedforward<units::meters> feedfwd(ks * 1.0_V, kv * 1_V * 1_s / 1_m, ka * 1_V * 1_s * 1_s / 1_m);

//Intake, Ball runs
static const int m_intakeFrontID = 9, m_intakeBackID = 7, m_lowerTowerID = 6, m_indexerID = 5;
rev::CANSparkMax m_intakeFrontMotor{m_intakeFrontID, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax m_intakeBackMotor{m_intakeBackID, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax m_lowerTowerMotor{m_lowerTowerID, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax m_indexerMotor{m_indexerID, rev::CANSparkMax::MotorType::kBrushed};
int indexerCounter = 0;
double feedspeed = 0;
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

//hood
static const int m_hoodID = 15;
rev::CANSparkMax m_hoodMotor{m_hoodID, rev::CANSparkMax::MotorType::kBrushless};
rev::SparkMaxRelativeEncoder m_hoodEncoder = m_hoodMotor.GetEncoder();
rev::SparkMaxPIDController m_hoodPID = m_hoodMotor.GetPIDController();
double targetHoodRotations = 0;
int hoodPreset = 0;
bool spinningTo = false;
frc::DigitalInput hoodLimit(0);

int autonChoice = 0;

double flywheelPcttoRPM(double pct){
  double rpm = 2048 * flywheelMaxRPM * pct / 600;
  return rpm;
}

double getLeftEncoderDist(){
  return m_leftEncoder.GetPosition() / driveGearRatio * (2 * M_PI * 0.0762)/*dist per rev in meters*/;
}

double getRightEncoderDist(){
  return m_rightEncoder.GetPosition() / driveGearRatio * (2 * M_PI * 0.0762);
}

void Robot::RobotInit() {
  cs::UsbCamera limelight = CameraServer::GetInstance()->StartAutomaticCapture(0);
  limelight.SetResolution(200, 150);
  limelight.SetFPS(40);
  _compressor.EnableDigital();
  
  m_leftFollowMotor.Follow(m_leftLeadMotor, false);
  m_rightFollowMotor.Follow(m_rightLeadMotor, false);

  m_leftLeadMotor.SetInverted(true);

  m_leftLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_leftFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_rightLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_rightFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

  m_rightEncoder.SetPosition(0.0);
  m_leftEncoder.SetPosition(0.0);

  m_intakeBackMotor.SetInverted(true);
  m_intakeFrontMotor.SetInverted(true);
  m_indexerMotor.SetInverted(true);

  /*m_leftWhinchMotor.SetInverted(false);
  m_rightWhinchMotor.SetInverted(false);*/
  m_rightWhinchMotor.Follow(m_leftWhinchMotor, true);
  m_leftWhinchMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightWhinchMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_hoodMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  _gyro.SetFusedHeading(0);

  m_flywheelMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0, 0);
  m_flywheelMotor.ConfigClosedloopRamp(1.5);
  m_flywheelMotor.SetSensorPhase(true);
  m_flywheelMotor.ConfigNominalOutputForward(0, 0);
  m_flywheelMotor.ConfigNominalOutputReverse(0, 0);
  m_flywheelMotor.ConfigPeakOutputForward(1, 0);
  m_flywheelMotor.ConfigPeakOutputReverse(-1, 0);

  m_flywheelMotor.Config_kF(0, 0.05, 0);
  m_flywheelMotor.Config_kP(0, 0.1, 0);
  m_flywheelMotor.Config_kI(0, 0.0, 0);
  m_flywheelMotor.Config_kD(0, 0.0, 0);

  backCam = frc::CameraServer::StartAutomaticCapture();
  frontCam = frc::CameraServer::StartAutomaticCapture();
  cameraSelection = nt::NetworkTableInstance::GetDefault().GetTable("")->GetEntry("CameraSelection");

  SmartDashboard::PutNumber("Min Belt Percent", 0.5);
  SmartDashboard::PutNumber("Min Intake Percent", 0.5);
  SmartDashboard::PutNumber("Min Indexer Percent", 0.5);
  SmartDashboard::PutNumber("Flywheel Far RPM", 5742 * flywheelGearRatio * 0.95);
  SmartDashboard::PutNumber("Flywheel Tarmac RPM", 5742 * flywheelGearRatio * 0.8);
  SmartDashboard::PutNumber("Flywheel Fender RPM", 5742 * flywheelGearRatio * 0.6);
  SmartDashboard::PutNumber("Whinch Percent", 0.5);
  SmartDashboard::PutNumber("Auton Choice", 2);

  sol_frontIntakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Robot::RobotPeriodic() {
  
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
  autonChoice = SmartDashboard::GetNumber("Auton Choice", 2);
}

void Robot::AutonomousPeriodic() {
  if (autonChoice == 0){
    switch(autonState){
      case 0:
        sol_frontIntakeSolenoid.Set(DoubleSolenoid::Value::kForward);
        m_intakeBackMotor.Set(-0.90);
        m_intakeFrontMotor.Set(-0.90);
        m_drive.ArcadeDrive(0.5, 0);
        m_flywheelMotor.Set(ControlMode::Velocity, flywheelPcttoRPM(0.74));
        SmartDashboard::PutNumber("Encoder", m_leftEncoder.GetPosition());
        if (m_leftEncoder.GetPosition() >= ((60/(6*M_PI))*8.68)){
          autonState += 1;
          m_leftEncoder.SetPosition(0);
          m_rightEncoder.SetPosition(0);
          m_leftLeadMotor.Disable();
          m_rightLeadMotor.Disable();
          _gyro.SetFusedHeading(0);
          m_lowerTowerMotor.Set(0.85);
          Wait(units::time::second_t(0.25));
          m_lowerTowerMotor.Set(0);
          Wait(units::time::second_t(0.5));
        }
        break;
      case 1:
        Wait(units::time::second_t(0.5));
        m_indexerMotor.Set(-0.75);
        Wait(units::time::second_t(1.0));
        m_flywheelMotor.Set(ControlMode::Velocity, flywheelPcttoRPM(0.72));
        m_lowerTowerMotor.Set(0.85);
        Wait(units::time::second_t(1.0));
        autonState += 1;
        //m_flywheelMotor.Set(0);
        m_indexerMotor.Set(0);
        m_lowerTowerMotor.Set(0);
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        m_hoodEncoder.SetPosition(0);
        _gyro.SetFusedHeading(0);
        break;
      case 2:
        m_drive.ArcadeDrive(0, 0.5);
        if (_gyro.GetFusedHeading() >= 28){
          autonState += 1;
          m_leftEncoder.SetPosition(0);
          m_rightEncoder.SetPosition(0);
          m_leftLeadMotor.Disable();
          m_rightLeadMotor.Disable();
          _gyro.SetFusedHeading(0);
        }
        break;
      case 3:
        m_drive.ArcadeDrive(-0.5, 0);
        if (m_leftEncoder.GetPosition() <= ((-120/(6*M_PI))*8.68)){
          autonState += 1;
          m_leftEncoder.SetPosition(0);
          m_rightEncoder.SetPosition(0);
          m_leftLeadMotor.Disable();
          m_rightLeadMotor.Disable();
          _gyro.SetFusedHeading(0);
          m_lowerTowerMotor.Set(0.85);
          Wait(units::time::second_t(0.25));
          m_lowerTowerMotor.Set(0);
          Wait(units::time::second_t(0.5));
        }
        break;
      case 4:
        m_flywheelMotor.Set(ControlMode::Velocity, flywheelPcttoRPM(0.70));
        m_drive.ArcadeDrive(0, -0.5);
        if (_gyro.GetFusedHeading() <= -91){
          autonState += 1;
          m_leftEncoder.SetPosition(0);
          m_rightEncoder.SetPosition(0);
          m_leftLeadMotor.Disable();
          m_rightLeadMotor.Disable();
          _gyro.SetFusedHeading(0);
        }
        break;
      case 5:
        Wait(units::time::second_t(0.5));
        m_indexerMotor.Set(-0.75);
        autonState += 1;
        m_flywheelMotor.Set(0);
        m_indexerMotor.Set(0);
        m_lowerTowerMotor.Set(0);
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        m_hoodEncoder.SetPosition(0);
        _gyro.SetFusedHeading(0);
        break;
      case 6:
        m_drive.ArcadeDrive(0, -0.5);
        if (_gyro.GetFusedHeading() <= -13){
          autonState += 1;
          m_leftEncoder.SetPosition(0);
          m_rightEncoder.SetPosition(0);
          m_leftLeadMotor.Disable();
          m_rightLeadMotor.Disable();
          _gyro.SetFusedHeading(0);
        }
        break;
      case 7:
        m_drive.ArcadeDrive(0.5, 0.0);
        if (m_leftEncoder.GetPosition() >= ((150/(6*M_PI))*8.68)){
          autonState += 1;
          m_leftEncoder.SetPosition(0);
          m_rightEncoder.SetPosition(0);
          m_leftLeadMotor.Disable();
          m_rightLeadMotor.Disable();
          _gyro.SetFusedHeading(0);
          m_lowerTowerMotor.Set(0.85);
          Wait(units::time::second_t(2));
          m_lowerTowerMotor.Set(0);
        }
        break;
      case 8:
        m_drive.ArcadeDrive(-0.5, 0);
        if (m_leftEncoder.GetPosition() <= ((-150/(6*M_PI))*8.68)){
          autonState += 1;
          m_leftEncoder.SetPosition(0);
          m_rightEncoder.SetPosition(0);
          m_leftLeadMotor.Disable();
          m_rightLeadMotor.Disable();
          _gyro.SetFusedHeading(0);
        }
        break;
      case 9:
        m_flywheelMotor.Set(ControlMode::Velocity, flywheelPcttoRPM(0.74));
        m_drive.ArcadeDrive(0, 0.5);
        if (_gyro.GetFusedHeading() >= 13){
          autonState += 1;
          m_leftEncoder.SetPosition(0);
          m_rightEncoder.SetPosition(0);
          m_leftLeadMotor.Disable();
          m_rightLeadMotor.Disable();
          _gyro.SetFusedHeading(0);
        }
        break;
      case 10:
        Wait(units::time::second_t(0.5));
        m_indexerMotor.Set(-0.75);
        Wait(units::time::second_t(1.0));
        m_flywheelMotor.Set(ControlMode::Velocity, flywheelPcttoRPM(0.72));
        m_lowerTowerMotor.Set(0.85);
        Wait(units::time::second_t(1.0));
        autonState += 1;
        //m_flywheelMotor.Set(0);
        m_indexerMotor.Set(0);
        m_lowerTowerMotor.Set(0);
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        m_hoodEncoder.SetPosition(0);
        _gyro.SetFusedHeading(0);
        break;
    }
  }
  else if (autonChoice == 1){
    switch(autonState){
      case 0:
        sol_frontIntakeSolenoid.Set(DoubleSolenoid::Value::kForward);
        m_intakeBackMotor.Set(-0.90);
        m_intakeFrontMotor.Set(-0.90);
        m_drive.ArcadeDrive(0.5, 0);
        m_flywheelMotor.Set(ControlMode::Velocity, flywheelPcttoRPM(0.74));
        SmartDashboard::PutNumber("Encoder", m_leftEncoder.GetPosition());
        if (m_leftEncoder.GetPosition() >= ((70/(6*M_PI))*8.68)){
          autonState += 1;
          m_leftEncoder.SetPosition(0);
          m_rightEncoder.SetPosition(0);
          m_leftLeadMotor.Disable();
          m_rightLeadMotor.Disable();
          _gyro.SetFusedHeading(0);
          m_lowerTowerMotor.Set(0.85);
          Wait(units::time::second_t(0.25));
          m_lowerTowerMotor.Set(0);
          Wait(units::time::second_t(0.5));
        }
        break;
      case 1:
        Wait(units::time::second_t(0.5));
        m_indexerMotor.Set(-0.75);
        Wait(units::time::second_t(1.0));
        m_flywheelMotor.Set(ControlMode::Velocity, flywheelPcttoRPM(0.72));
        m_lowerTowerMotor.Set(0.85);
        Wait(units::time::second_t(1.0));
        autonState += 1;
        //m_flywheelMotor.Set(0);
        m_indexerMotor.Set(0);
        m_lowerTowerMotor.Set(0);
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        m_hoodEncoder.SetPosition(0);
        _gyro.SetFusedHeading(0);
        break;
      case 2:
        m_drive.ArcadeDrive(0, 0.5);
        if (_gyro.GetFusedHeading() >= 37){
          autonState += 1;
          m_leftEncoder.SetPosition(0);
          m_rightEncoder.SetPosition(0);
          m_leftLeadMotor.Disable();
          m_rightLeadMotor.Disable();
          _gyro.SetFusedHeading(0);
        }
        break;
      case 3:
        m_drive.ArcadeDrive(-0.5, 0);
        if (m_leftEncoder.GetPosition() <= ((-120/(6*M_PI))*8.68)){
          autonState += 1;
          m_leftEncoder.SetPosition(0);
          m_rightEncoder.SetPosition(0);
          m_leftLeadMotor.Disable();
          m_rightLeadMotor.Disable();
          _gyro.SetFusedHeading(0);
          m_lowerTowerMotor.Set(0.85);
          Wait(units::time::second_t(0.25));
          m_lowerTowerMotor.Set(0);
          Wait(units::time::second_t(0.5));
        }
        break;
      case 4:
        m_flywheelMotor.Set(ControlMode::Velocity, flywheelPcttoRPM(0.70));
        m_drive.ArcadeDrive(0, -0.5);
        if (_gyro.GetFusedHeading() <= -91){
          autonState += 1;
          m_leftEncoder.SetPosition(0);
          m_rightEncoder.SetPosition(0);
          m_leftLeadMotor.Disable();
          m_rightLeadMotor.Disable();
          _gyro.SetFusedHeading(0);
        }
        break;
      case 5:
        Wait(units::time::second_t(0.5));
        m_indexerMotor.Set(-0.75);
        Wait(units::time::second_t(1.0));
        m_lowerTowerMotor.Set(0.85);
        Wait(units::time::second_t(1.0));
        autonState += 1;
        m_flywheelMotor.Set(0);
        m_indexerMotor.Set(0);
        m_lowerTowerMotor.Set(0);
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        m_hoodEncoder.SetPosition(0);
        _gyro.SetFusedHeading(0);
        break;
    }
  }
  else if (autonChoice == 2){
    switch (autonState){
      case 0:
        sol_frontIntakeSolenoid.Set(DoubleSolenoid::Value::kForward);
        m_intakeBackMotor.Set(-0.90);
        m_intakeFrontMotor.Set(-0.90);
        m_flywheelMotor.Set(ControlMode::Velocity, flywheelPcttoRPM(0.62));
        m_drive.ArcadeDrive(0.5, 0);
        SmartDashboard::PutNumber("Auton State", autonState);
        SmartDashboard::PutNumber("Encoder", m_leftEncoder.GetPosition());
        if (m_leftEncoder.GetPosition() >= ((100/(6*M_PI))*8.68)){
          autonState += 1;
          m_leftEncoder.SetPosition(0);
          m_rightEncoder.SetPosition(0);
          m_leftLeadMotor.Disable();
          m_rightLeadMotor.Disable();
          _gyro.SetFusedHeading(0);
          m_lowerTowerMotor.Set(0.85);
          Wait(units::time::second_t(0.25));
          m_lowerTowerMotor.Set(0);
          Wait(units::time::second_t(0.5));
        }
        break;
      case 1:
        m_hoodMotor.Set(-0.5);
        if(m_hoodEncoder.GetPosition() < -180*0.95){
          m_hoodMotor.Set(0);
          autonState += 1;
        }
        break;
      case 2:
        SmartDashboard::PutNumber("Auton State", autonState);
        m_drive.ArcadeDrive(-0.5, 0);
        SmartDashboard::PutNumber("Encoder", m_leftEncoder.GetPosition());
        if (m_leftEncoder.GetPosition() <= ((-60/(6*M_PI))*8.68)){
          autonState += 1;
          m_leftEncoder.SetPosition(0);
          m_rightEncoder.SetPosition(0);
          m_leftLeadMotor.Disable();
          m_rightLeadMotor.Disable();
          _gyro.SetFusedHeading(0);
        }
        break;
      case 3:
        Wait(units::time::second_t(0.5));
        m_indexerMotor.Set(-0.85);
        Wait(units::time::second_t(2.0));
        m_lowerTowerMotor.Set(0.85);
        Wait(units::time::second_t(1.0));
        autonState += 1;
        m_flywheelMotor.Set(0);
        m_indexerMotor.Set(0);
        m_lowerTowerMotor.Set(0);
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        m_hoodEncoder.SetPosition(0);
        _gyro.SetFusedHeading(0);
        break;
      case 4:
        m_hoodMotor.Set(0.75);
        if (hoodLimit.Get()){
          m_hoodMotor.Set(0);
          m_hoodEncoder.SetPosition(0);
          autonState += 1;
        }
        break;
    }
  }
  else if (autonChoice == 3){
    switch (autonState){
      case 0:
        m_flywheelMotor.Set(ControlMode::Velocity, flywheelPcttoRPM(0.6));
        Wait(units::time::second_t(0.5));
        m_indexerMotor.Set(-0.75);
        Wait(units::time::second_t(1.0));
        autonState += 1;
        m_flywheelMotor.Set(0);
        m_indexerMotor.Set(0);
        m_lowerTowerMotor.Set(0);
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        m_hoodEncoder.SetPosition(0);
        _gyro.SetFusedHeading(0);
        break;
      case 1:
        m_drive.ArcadeDrive(0.5, 0.0);
        if (m_leftEncoder.GetPosition() >= ((125/(6*M_PI))*8.68)){
          autonState += 1;
          m_leftEncoder.SetPosition(0);
          m_rightEncoder.SetPosition(0);
          m_leftLeadMotor.Disable();
          m_rightLeadMotor.Disable();
          _gyro.SetFusedHeading(0);
        }
        break;
    }
  }
}

void Robot::TeleopInit() {
  m_leftLeadMotor.SetIdleMode(CANSparkMax::IdleMode::kCoast);
  m_rightLeadMotor.SetIdleMode(CANSparkMax::IdleMode::kCoast);
  m_leftFollowMotor.SetIdleMode(CANSparkMax::IdleMode::kCoast);
  m_rightFollowMotor.SetIdleMode(CANSparkMax::IdleMode::kCoast);
  m_indexerMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
  m_hoodEncoder.SetPosition(0);
}

void Robot::TeleopPeriodic() {
  SmartDashboard::PutNumber("Encoder", m_leftEncoder.GetPosition());
  SmartDashboard::PutNumber("Hood Encoder", m_hoodEncoder.GetPosition());
  /*
  */
  if (cont_Partner->GetPOV() == 90){
    spinningTo = false;
    m_hoodMotor.Set(-0.5);
  }
  else if (cont_Partner->GetPOV() == 270){
    spinningTo = false;
    if (hoodLimit.Get()){
      m_hoodMotor.Set(0);
      m_hoodEncoder.SetPosition(0);
    }
    else{
      m_hoodMotor.Set(0.5);
    }
  }
  else if (spinningTo){
    if (m_hoodEncoder.GetPosition() < targetHoodRotations*1.05){
      m_hoodMotor.Set(0.5);
    }
    else if (m_hoodEncoder.GetPosition() > targetHoodRotations*0.95){
      m_hoodMotor.Set(-0.5);
    }
    else{
      spinningTo = false;
      m_hoodMotor.Disable();
    }
  }
  else if(!spinningTo){
    m_hoodMotor.Disable();
  }
  

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
  }
  else if (cont_Driver->GetPOV() == 270){
    sol_climber.Set(frc::DoubleSolenoid::Value::kReverse);
  }

  if (cont_Driver->GetPOV() == 180){
    m_leftWhinchMotor.Set(0.8);
  }
  else if (cont_Driver->GetPOV() == 0 && cont_Driver->IsConnected()){
    m_leftWhinchMotor.Set(-0.8);
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
  if (cont_Partner->GetR2ButtonPressed()){
    indexerCounter = 0;
  }
  else if (cont_Partner->GetR2Button()){
    indexerCounter += 1;
    if (indexerCounter < 8){
      m_indexerMotor.Set(-0.75);
      //m_lowerTowerMotor.Set(0.75);
      feedspeed = 0.75;
    }
    else{
      m_indexerMotor.Disable();
      feedspeed = 0;
    }
  }
  else if (cont_Partner->GetR2ButtonReleased()){
    feedspeed = 0;
    m_indexerMotor.Disable();
  }
  else if (cont_Partner->GetL2Button()){
    m_indexerMotor.Set(0.75);
  }
  else {
    m_indexerMotor.Disable();
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
  if (cont_Driver->GetR1Button()){
    m_lowerTowerMotor.Set(0.75);
  }
  else if (cont_Driver->GetL1Button()){
    m_lowerTowerMotor.Set(-0.75);
  }
  else {
    m_lowerTowerMotor.Set(feedspeed);
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
  if (cont_Driver->GetR2ButtonPressed()){
    sol_frontIntakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
  }
  else if (cont_Driver->GetR2ButtonReleased()){
    sol_frontIntakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
  }
  if (cont_Driver->GetR2Button()){
    m_intakeFrontMotor.Set(-0.75);
    m_intakeBackMotor.Set(-0.75);
    m_lowerTowerMotor.Set(0.75);
  }
  else if (cont_Driver->GetL2Button()){
    m_intakeBackMotor.Set(0.75);
    m_intakeFrontMotor.Set(0.75);
    m_lowerTowerMotor.Set(-0.75);
  }
  else if (cont_Driver->GetTriangleButton()){
    m_intakeBackMotor.Set(-0.75);
    m_intakeFrontMotor.Set(-0.75);
  }
  else {
    m_intakeBackMotor.Set(-0.2);
    m_intakeFrontMotor.Set(-0.2);
  }

  if (cont_Driver->GetCircleButtonPressed()){
    sol_frontIntakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
  }
  else if (cont_Driver->GetCrossButtonPressed()){
    sol_frontIntakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
  }

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
  if (cont_Partner->GetTriangleButtonPressed()){
    flywheelTargetPCT = 0.57;
    targetHoodRotations = -5;
    spinningTo = true;
    m_flywheelMotor.Set(motorcontrol::ControlMode::Velocity, flywheelPcttoRPM(flywheelTargetPCT));
  }
  else if (cont_Partner->GetSquareButtonPressed()){
    flywheelTargetPCT = 0.35;
    targetHoodRotations = -5;
    spinningTo = true;
    m_flywheelMotor.Set(motorcontrol::ControlMode::Velocity, flywheelPcttoRPM(flywheelTargetPCT));
  }
  else if (cont_Partner->GetCircleButtonPressed()){
    flywheelTargetPCT = 0.60;
    targetHoodRotations = -180;
    spinningTo = true;
    m_flywheelMotor.Set(motorcontrol::ControlMode::Velocity, flywheelPcttoRPM(flywheelTargetPCT));
  }
  else if (cont_Partner->GetCrossButtonPressed()){
    flywheelTargetPCT = 0.85;
    targetHoodRotations = -180;
    spinningTo = true;
    m_flywheelMotor.Set(motorcontrol::ControlMode::Velocity, flywheelPcttoRPM(flywheelTargetPCT));
  }
  else if (cont_Partner->GetPSButtonPressed()){
    flywheelTargetPCT = 0;
    m_flywheelMotor.Set(motorcontrol::ControlMode::Velocity, flywheelTargetPCT);
  }
  else if (cont_Partner->GetR1ButtonPressed()){
    flywheelTargetPCT += 0.05;
    m_flywheelMotor.Set(motorcontrol::ControlMode::Velocity, flywheelPcttoRPM(flywheelTargetPCT));
  }
  else if (cont_Partner->GetL1ButtonPressed()){
    flywheelTargetPCT -= 0.05;
    m_flywheelMotor.Set(motorcontrol::ControlMode::Velocity, flywheelPcttoRPM(flywheelTargetPCT));
  }
  SmartDashboard::PutNumber("Target PCT", flywheelTargetPCT);

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

  if (abs(joy_rStick_Y) < joy_rStick_Y_deadband){
    joy_rStick_Y = 0;
  }

  if (abs(joy_rStick_X) < joy_rStick_X_deadband){
    joy_rStick_X = 0;
  }

  if (joy_lStick_Y >= 0){
    cameraSelection.SetString(frontCam.GetName());
  }
  else {
    cameraSelection.SetString(backCam.GetName());
  }

  if (cont_Driver->GetTouchpad()){
    double kp = -0.02;
    double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
    if (tx > 2.5){
      m_drive.ArcadeDrive(joy_lStick_Y, kp*tx);
    }
    else if (tx < -2.5){
      m_drive.ArcadeDrive(joy_lStick_Y, kp*tx);
    }
    else{
      m_drive.ArcadeDrive(joy_lStick_Y, 0);
    }
  }
  else{
    m_drive.ArcadeDrive(joy_lStick_Y*0.8, -joy_rStick_X*0.8);
  }


  //Special Commands
  if (cont_Partner->GetTouchpad()){
    double ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
    flywheelTargetRPM = flywheelPcttoRPM(2*((ty*-0.008)+0.3));
    m_flywheelMotor.Set(ControlMode::Velocity, flywheelTargetRPM);
    double targetHood = ((ty*3.1011)-15.210);
    if (m_hoodEncoder.GetPosition() > targetHood*0.95){
      m_hoodMotor.Set(-0.5);
    }
    else if(m_hoodEncoder.GetPosition() < targetHood*1.05){
      m_hoodMotor.Set(0.5);
    }
    else{
      m_hoodMotor.Set(0);
    }
  }
}
void Robot::DisabledInit() {
  m_leftLeadMotor.SetIdleMode(CANSparkMax::IdleMode::kCoast);
  m_leftFollowMotor.SetIdleMode(CANSparkMax::IdleMode::kCoast);
  m_rightLeadMotor.SetIdleMode(CANSparkMax::IdleMode::kCoast);
  m_rightFollowMotor.SetIdleMode(CANSparkMax::IdleMode::kCoast);
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
