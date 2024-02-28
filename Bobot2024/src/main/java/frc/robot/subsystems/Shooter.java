// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix6.StatusCode;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Shooter extends SubsystemBase {
  private final TalonFX pivot = new TalonFX(Constants.ShooterConstants.pivotShooterMotorID);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  private final CANSparkMax leftShoot = new CANSparkMax(Constants.ShooterConstants.leftShootMotorID, MotorType.kBrushless);  
  private final CANSparkMax rightShoot = new CANSparkMax(Constants.ShooterConstants.rightShootMotorID, MotorType.kBrushless);
  private final CANSparkMax retainer =  new CANSparkMax(Constants.ShooterConstants.retainerMotorID, MotorType.kBrushless);
  private final DigitalInput infraSensor =  new DigitalInput(1);
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, AmpRPM, SpkRPM;

  /** Creates a new Shooter. */
  public Shooter() {
    leftShoot.restoreFactoryDefaults();
    m_pidController = leftShoot.getPIDController();
    m_encoder = leftShoot.getEncoder();
    kP = Constants.ShooterConstants.velP;//0.000654 
    kI = Constants.ShooterConstants.velI;
    kD = Constants.ShooterConstants.velD; 
    kIz = Constants.ShooterConstants.velIz; 
    kFF = Constants.ShooterConstants.velFF; 
    kMaxOutput = Constants.ShooterConstants.velMaxOut; 
    kMinOutput = Constants.ShooterConstants.velMinOut;
    AmpRPM = Constants.ShooterConstants.AmpVel;
    SpkRPM = Constants.ShooterConstants.SpeakerVel;

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    TalonFXConfiguration configs = new TalonFXConfiguration();

    MotionMagicConfigs mm = configs.MotionMagic;
    mm.MotionMagicCruiseVelocity = 170; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 200; // Take approximately 0.5 seconds to reach max vel
    // Take approximately 0.2 seconds to reach max accel 
    mm.MotionMagicJerk = 1000;

    Slot0Configs slot0 = configs.Slot0;

    slot0.kP = Constants.ShooterConstants.kP;
    slot0.kI = Constants.ShooterConstants.kI;
    slot0.kD = Constants.ShooterConstants.kD;
    slot0.kV = 0.12;
    slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving

    FeedbackConfigs fdb = configs.Feedback;
    fdb.SensorToMechanismRatio = 12.8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      status = pivot.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    //pivot.setPosition(0);
    
    rightShoot.follow(leftShoot, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Encoder", pivot.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Power", pivot.getAcceleration().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Velocity", m_encoder.getVelocity());
    SmartDashboard.putNumber("Shooter Voltage", pivot.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Current", pivot.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("Sensor Shooter", infraSensor.get());
  }

  public void shoot() {
    leftShoot.set(-1);
  }

  public void eject() {
    m_pidController.setReference(AmpRPM, CANSparkMax.ControlType.kVelocity);
  }

  public void stop() {
    leftShoot.set(0);
  }

  public void stopRetainer() {
    retainer.set(0);
  }

  public void soltar() {
    retainer.set(-0.5);
  }

  public void receive() {
    retainer.set(0.5);
    leftShoot.set(0.25);
  }

  public void setPosition(double pos) {
    pivot.setControl(m_mmReq.withPosition(pos));
  }

  public void manualControl(double val) {
    pivot.set(val);
  }

  public boolean getSensor() {
    return infraSensor.get();
  }

  public double shareEncoder(){
    return pivot.getPosition().getValueAsDouble();
  }

}