// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Constants;
import frc.robot.subsystems.*;

public class LimelightTurretSwerve extends Command {
  /** Creates a new LimelightTurretSwerve. */
  private Swerve s_Swerve;
  private Limelight l_Limelight;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  public PIDController limelightPID;

  public LimelightTurretSwerve(
    Limelight l_Limelight,
    Swerve s_Swerve,
    DoubleSupplier translationSup,
    DoubleSupplier strafeSup,
    DoubleSupplier rotationSup,
    BooleanSupplier robotCentricSup
  ) {
    addRequirements(s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
    this.l_Limelight = l_Limelight;
    this.s_Swerve = s_Swerve;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    l_Limelight.SelectPipeline(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("Turret Active");
    double translationVal = (Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 1));
    double strafeVal = (Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 1));
    double rotationVal = (Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband), 1));

   limelightPID = new PIDController(0.01, 0.00008, 0);
               
    
   

    if(l_Limelight.getArea()>0){ 
      s_Swerve.drive(
      new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
      limelightPID.calculate(l_Limelight.getTX(), 0) * Constants.Swerve.maxAngularVelocity, 
      !robotCentricSup.getAsBoolean(), 
      true);
    }
    else{
        s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
        rotationVal * Constants.Swerve.maxAngularVelocity, 
        !robotCentricSup.getAsBoolean(), 
        true);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
