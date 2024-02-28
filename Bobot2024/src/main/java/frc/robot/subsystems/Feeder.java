// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.wpilibj.Joystick;


public class Feeder extends SubsystemBase {
  private final CANSparkMax pivotLeft = new CANSparkMax(Constants.FeederConstants.feederLeftPivotMotorID, MotorType.kBrushed);
  private final CANSparkMax pivotRight = new CANSparkMax(Constants.FeederConstants.feederRightPivotMotorID, MotorType.kBrushed);
  private final CANSparkMax gripper = new CANSparkMax(Constants.FeederConstants.gripperID, MotorType.kBrushless);
  private final DigitalInput infraSensor = new DigitalInput(0);
  private final Joystick driver = new Joystick(0);
  private RelativeEncoder m_EncoderLeft;
  private RelativeEncoder m_EncoderRight;
  private SparkPIDController PID_PivotControlLeft;
  private SparkPIDController PID_PivotControlRight;

  /** Creates a new Feeder. */
  public Feeder() {
    pivotLeft.restoreFactoryDefaults();
    pivotRight.restoreFactoryDefaults();
    gripper.restoreFactoryDefaults();

    pivotLeft.setInverted(true);
    m_EncoderLeft = pivotLeft.getEncoder(SparkRelativeEncoder.Type.kQuadrature,8092);
    PID_PivotControlLeft = pivotLeft.getPIDController();
    PID_PivotControlLeft.setFeedbackDevice(m_EncoderLeft);

    PID_PivotControlLeft.setP(Constants.FeederConstants.pivotKP);
    PID_PivotControlLeft.setI(Constants.FeederConstants.pivotKI);
    PID_PivotControlLeft.setD(Constants.FeederConstants.pivotKD);
    PID_PivotControlLeft.setIZone(Constants.FeederConstants.pivotKIz);
    PID_PivotControlLeft.setFF(Constants.FeederConstants.pivotKFF);
    PID_PivotControlLeft.setOutputRange(Constants.FeederConstants.KMinOutput, Constants.FeederConstants.kMaxOutput);

    pivotRight.setInverted(true);
    m_EncoderRight = pivotRight.getEncoder(SparkRelativeEncoder.Type.kQuadrature,8092);
    PID_PivotControlRight = pivotRight.getPIDController();
    PID_PivotControlRight.setFeedbackDevice(m_EncoderRight);

    PID_PivotControlRight.setP(Constants.FeederConstants.pivotKP);
    PID_PivotControlRight.setI(Constants.FeederConstants.pivotKI);
    PID_PivotControlRight.setD(Constants.FeederConstants.pivotKD);
    PID_PivotControlRight.setIZone(Constants.FeederConstants.pivotKIz);
    PID_PivotControlRight.setFF(Constants.FeederConstants.pivotKFF);
    PID_PivotControlRight.setOutputRange(Constants.FeederConstants.KMinOutput, Constants.FeederConstants.kMaxOutput);

    int smartMotionLeft = 1;
    PID_PivotControlLeft.setSmartMotionMaxVelocity(Constants.FeederConstants.maxVel, smartMotionLeft);
    PID_PivotControlLeft.setSmartMotionMinOutputVelocity(Constants.FeederConstants.minVel, smartMotionLeft);
    PID_PivotControlLeft.setSmartMotionMaxAccel(Constants.FeederConstants.maxAcc, smartMotionLeft);
    PID_PivotControlLeft.setSmartMotionAllowedClosedLoopError(Constants.FeederConstants.allowedErr, smartMotionLeft);

    int smartMotionRight = 2;
    PID_PivotControlRight.setSmartMotionMaxVelocity(Constants.FeederConstants.maxVel, smartMotionRight);
    PID_PivotControlRight.setSmartMotionMinOutputVelocity(Constants.FeederConstants.minVel, smartMotionRight);
    PID_PivotControlRight.setSmartMotionMaxAccel(Constants.FeederConstants.maxAcc, smartMotionRight);
    PID_PivotControlRight.setSmartMotionAllowedClosedLoopError(Constants.FeederConstants.allowedErr, smartMotionRight);

  }

  public void dropFeeder() {
    gripper.set(-1);
    PID_PivotControlLeft.setReference(Constants.FeederConstants.eatLeftPosition, ControlType.kPosition);
    PID_PivotControlRight.setReference(Constants.FeederConstants.eatRightPosition, ControlType.kPosition);
  }

  public void rumble() {
    driver.setRumble(RumbleType.kBothRumble, 1);
  }

  public void norumble() {
    driver.setRumble(RumbleType.kBothRumble, 0);
  }

  public boolean getSensor() {
    return infraSensor.get();
  }

  public void ampPosition() {
    //PID_PivotControlLeft.setReference(Constants.FeederConstants.ampLeftPosition, ControlType.kPosition);
    //PID_PivotControlRight.setReference(Constants.FeederConstants.ampRightPosition, ControlType.kPosition);
    gripper.set(0.45);
  }

  public void handoffPosition() {
    PID_PivotControlLeft.setReference(Constants.FeederConstants.handoffLeftPosition, ControlType.kPosition);
    PID_PivotControlRight.setReference(Constants.FeederConstants.handoffRightPosition, ControlType.kPosition);
  }

  public void passThrough() {
    gripper.set(-0.8);
  }

  public void saveFeeder() {
    gripper.set(0);
    PID_PivotControlLeft.setReference(Constants.FeederConstants.initLeftPosition, ControlType.kPosition);
    PID_PivotControlRight.setReference(Constants.FeederConstants.initRightPosition, ControlType.kPosition);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Pivot Encoder", m_EncoderLeft.getPosition());   
    SmartDashboard.putNumber("Right Pivot Encoder", m_EncoderRight.getPosition());
    SmartDashboard.putBoolean("Sensor Feeder", infraSensor.get());
  }
}
