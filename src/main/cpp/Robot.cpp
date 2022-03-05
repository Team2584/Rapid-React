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

#include <frc/drive/differentialdrive.h>
#include "networktables/NetworkTable.h"
#include <networktables/NetworkTableEntry.h>
#include <networktables/EntryListenerFlags.h>
#include "networktables/NetworkTableInstance.h"
#include "cameraserver/CameraServer.h"

#include <frc/drive/differentialdrive.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/EntryListenerFlags.h>


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
//rev::SparkMaxRelativeEncoder m_leftEncoderABS = m_leftLeadMotor.GetEncoder();
rev::SparkMaxRelativeEncoder m_rightEncoder = m_rightLeadMotor.GetEncoder();
//rev::SparkMaxRelativeEncoder m_rightEncoderABS = m_rightLeadMotor.GetEncoder();

frc::DifferentialDrive m_drive{m_leftLeadMotor, m_rightLeadMotor};
frc2::PIDController driveControl{drivekP, drivekI, drivekD};

WPI_PigeonIMU _gyro(0);
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
double flywheelTargetPCT = 0;

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

double flywheelPcttoRPM(double pct){
  double rpm = 4096 * flywheelMaxRPM * pct / 600;
  return rpm;
}

double getHoodRotationsLimelight(double ty){
  return ty * 50.0/*hood const*/;
}

double getHoodRotationsDistance(double dist){
  return dist *20;
}

double flywheelPCTLimelight(double ty){
  return ty * 4.0 + 0.6;
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
  //m_rightEncoderABS.SetPosition(0);
  m_leftEncoder.SetPosition(0.0);
  //m_leftEncoderABS.SetPosition(0);

  m_intakeBackMotor.SetInverted(true);
  m_intakeFrontMotor.SetInverted(true);
  m_indexerMotor.SetInverted(true);

  m_rightWhinchMotor.Follow(m_leftWhinchMotor);
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

  // Values for limelight
  float KpX = -0.02;
  float KpY = -0.01;

  double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
  double ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
  double ta = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta", 0.0);
}

void Robot::RobotPeriodic() {
  /*position = odometry.Update(getGyroAngle(), units::meter_t(getLeftEncoderDist()), units::meter_t(getRightEncoderDist()));
  autoAimPID.SetSetpoint(0);
  double ty = 
  targetHoodRotations = getHoodRotationsLimelight(nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", targetHoodRotations));
  double targetBasedDist = getHoodRotationsDistance(sqrt(pow(position.X().value(), 2.0) + pow(position.Y().value(), 2.0)));
  double targetFlywheelRPMLimelight = flywheelPcttoRPM(flywheelPCTLimelight(nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0)));
  */
  /*if (m_hoodEncoder.GetPosition() > targetHoodRotations*.9){
    m_hoodMotor.Set(-0.5);
  }
  else if(m_hoodEncoder.GetPosition() < targetHoodRotations*1.1){
    m_hoodMotor.Set(0.5);
  }
  else{
    m_hoodMotor.Disable();
  }*/
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
  switch(autonState){
    case 0:
      sol_frontIntakeSolenoid.Set(DoubleSolenoid::Value::kForward);
      m_intakeBackMotor.Set(-0.75);
      m_intakeFrontMotor.Set(-0.75);
      m_drive.ArcadeDrive(0.5, 0);
      SmartDashboard::PutNumber("Encoder", m_leftEncoder.GetPosition());
      if (m_leftEncoder.GetPosition() >= ((76/(6*M_PI))*8.68)){
        autonState += 1;
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        _gyro.SetFusedHeading(0);
        m_lowerTowerMotor.Set(0.6);
        Wait(units::time::second_t(0.25));
        m_lowerTowerMotor.Set(0);
        Wait(units::time::second_t(0.5));
      }
      break;
    case 1:
      m_flywheelMotor.Set(ControlMode::Velocity, flywheelPcttoRPM(0.3));
      m_drive.ArcadeDrive(-0.5, 0);
      if (m_leftEncoder.GetPosition() <= ((-90/(6*M_PI))*8.68)){
        autonState += 2;
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        _gyro.SetFusedHeading(0);
      }
      break;
    case 3:
      m_indexerMotor.Set(-0.75);
      Wait(units::time::second_t(1.0));
      m_lowerTowerMotor.Set(0.75);
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
      m_drive.ArcadeDrive(0, -0.5);
      if (_gyro.GetFusedHeading() <= -40){
        autonState += 1;
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        _gyro.SetFusedHeading(0);
      }
      break;
    case 5:
      m_drive.ArcadeDrive(0.5, 0);
      if (m_leftEncoder.GetPosition() >= ((90/(6*M_PI))*8.68)){
        autonState += 1;
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        _gyro.SetFusedHeading(0);
        m_lowerTowerMotor.Set(0.6);
        m_indexerMotor.Set(0.6);
        Wait(units::time::second_t(0.75));
        m_indexerMotor.Set(0);
        m_lowerTowerMotor.Set(0);
      }
      break;
    case 6:
      m_drive.ArcadeDrive(0, 0.5);
      if (_gyro.GetFusedHeading() >= 10){
        autonState += 1;
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        _gyro.SetFusedHeading(0);
      }
      break;
    case 7:
      m_flywheelMotor.Set(ControlMode::Velocity, flywheelPcttoRPM(0.3));
      m_drive.ArcadeDrive(-0.5, 0);
      if (m_leftEncoder.GetPosition() <= ((-60/(6*M_PI))*8.68)){
        autonState += 1;
        m_leftEncoder.SetPosition(0);
        m_rightEncoder.SetPosition(0);
        m_leftLeadMotor.Disable();
        m_rightLeadMotor.Disable();
        _gyro.SetFusedHeading(0);
      }
      break;
    case 8:
      m_indexerMotor.Set(-0.75);
      Wait(units::time::second_t(1.0));
      m_lowerTowerMotor.Set(0.75);
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

void Robot::TeleopInit() {
  m_leftLeadMotor.SetIdleMode(CANSparkMax::IdleMode::kCoast);
  m_rightLeadMotor.SetIdleMode(CANSparkMax::IdleMode::kCoast);
  m_leftFollowMotor.SetIdleMode(CANSparkMax::IdleMode::kCoast);
  m_rightFollowMotor.SetIdleMode(CANSparkMax::IdleMode::kCoast);

  m_flywheelMotor.ConfigFactoryDefault();
  m_flywheelMotor.SetSensorPhase(false);
  m_flywheelMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
}

void Robot::TeleopPeriodic() {
  SmartDashboard::PutNumber("Encoder", m_leftEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Falcon RPM:", (m_flywheelMotor.GetSelectedSensorVelocity(0)/2048)*600); 
  
  if (cont_Partner->GetPOV() == 90){
    m_hoodMotor.Set(-0.5);
  }
  else if (cont_Partner->GetPOV() == 270){
    m_hoodMotor.Set(0.5);
  }
  else{
    m_hoodMotor.Disable();
  }

  if (cont_Driver->GetPOV() == 0){
    tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
    ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
    ta = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta", 0.0);
    
    float KpX = -0.02;

    //Turn & Driving Tracking (SHOULD Align well enough to shoot)
    if(!(tx < 7.5 && tx > -7.5)){
      //float KpX = -0.03;
      float steering_adjust = KpX * tx;

      m_drive.TankDrive(-steering_adjust, -steering_adjust);
    } else if(!(tx < 2.5 && tx > -2.5)){

      // float KpX = -0.05;
      float steering_adjust = KpX * tx;

      m_drive.TankDrive(-(steering_adjust+0.0001), -(steering_adjust+0.0001));

    } else if ((tx < 3.5 && tx > -3.5) && (ta <= 0.007)){

      //Adjust ta (target area) value to be a good distance away from the hub to shoot and score
      m_drive.TankDrive(.22,-.22);

    } else if ((tx < 3.5 && tx > -3.5) && (ta >= 0.008)){

      //Adjust ta (target area) value to be a good distance away from the hub to shoot and score
      m_drive.TankDrive(-.22,.22);

    } else{

      m_drive.TankDrive(0, 0);

    }
    tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
    ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
    ta = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta", 0.0);
  }

  else if (cont_Driver->GetPOV() == 180){
    // Convert Encoder Values to RPM
    double normalizedRPM = (m_flywheelMotor.GetSelectedSensorVelocity(0)/2048)*600;

    flywheelTargetPCT = 0.268;
    m_flywheelMotor.Set(motorcontrol::ControlMode::Velocity, flywheelPcttoRPM(flywheelTargetPCT));

    // Spin flywheel at 3825 rpm. Flywheel 60% = 3750 rpm

    // mFeed.Set(centerIntake);
    if (normalizedRPM > 3745 && normalizedRPM < 3890){
      m_indexerMotor.Set(.5);
      m_lowerTowerMotor.Set(.22);
    }
    // mIndexer.Set(0);
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
  else if (cont_Driver->GetPOV() == 90){
    sol_climber.Set(frc::DoubleSolenoid::Value::kForward);
  }
  else if (cont_Driver->GetPOV() == 270){
    sol_climber.Set(frc::DoubleSolenoid::Value::kReverse);
  }

  else if (cont_Driver->GetPOV() == 0){
    m_leftWhinchMotor.Set(0.5);
    m_rightWhinchMotor.Set(0.5);
  }
  else if (cont_Driver->GetPOV() == 180){
    m_leftWhinchMotor.Set(-0.5);
    m_rightWhinchMotor.Set(-0.5);
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
  else if (cont_Partner->GetR2Button()){
    m_indexerMotor.Set(-0.75);
  }
  else if (cont_Partner->GetL2Button()){
    m_indexerMotor.Set(0.75);
  }
  else {
    m_indexerMotor.Set(0);
    m_leftWhinchMotor.Disable();
    m_rightWhinchMotor.Disable();
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
  if (cont_Driver->GetR2Button()){
    m_intakeFrontMotor.Set(-0.75);
    m_intakeBackMotor.Set(-0.75);
  }
  else if (cont_Driver->GetL2Button()){
    m_intakeBackMotor.Set(0.75);
    m_intakeFrontMotor.Set(0.75);
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
    flywheelTargetPCT = 0.3;
    m_flywheelMotor.Set(motorcontrol::ControlMode::Velocity, flywheelPcttoRPM(flywheelTargetPCT));
  }
  else if (cont_Partner->GetSquareButtonPressed()){
    flywheelTargetPCT = 0.55;
    m_flywheelMotor.Set(motorcontrol::ControlMode::Velocity, flywheelPcttoRPM(flywheelTargetPCT));
  }
  else if (cont_Partner->GetCircleButtonPressed()){
    flywheelTargetPCT = 0.85;
    m_flywheelMotor.Set(motorcontrol::ControlMode::Velocity, flywheelPcttoRPM(flywheelTargetPCT));
  }
  else if (cont_Partner->GetCrossButtonPressed()){
    flywheelTargetPCT = 0.6;
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
