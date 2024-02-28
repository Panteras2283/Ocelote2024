// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class PrecisionSwerve extends Command {
  /** Creates a new PrecisionSwerve. */
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier cardFront;
  private BooleanSupplier cardBack;
  private BooleanSupplier cardLeft;
  private BooleanSupplier cardRight;

  public PrecisionSwerve(
    Swerve s_Swerve, 
    DoubleSupplier translationSup, 
    DoubleSupplier strafeSup, 
    DoubleSupplier rotationSup, 
    BooleanSupplier robotCentricSup,
    BooleanSupplier cardFront,
    BooleanSupplier cardBack,
    BooleanSupplier cardLeft,
    BooleanSupplier cardRight
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.cardFront=cardFront;
    this.cardBack=cardBack;
    this.cardLeft=cardLeft;
    this.cardRight=cardRight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        double translationVal = (Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 1));
        double strafeVal = (Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 1));
        double rotationVal = (Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband), 1));
        double targetAngle=0;        

        System.out.println("Precision Active");

        
        /*Lock robot to specific angles (Disables rotation joystick while specific orientation is selected) */
        if(cardFront.getAsBoolean()==true || cardBack.getAsBoolean()==true||cardLeft.getAsBoolean()==true||cardRight.getAsBoolean()==true){
            if(cardFront.getAsBoolean()==true){
                targetAngle=0;
            }
            else if(cardBack.getAsBoolean()==true){
                targetAngle=180;
            }
            else if(cardLeft.getAsBoolean()==true){
                targetAngle=90;
            }
            else if(cardRight.getAsBoolean()==true){
                targetAngle=270;
            }
            s_Swerve.lockDrive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.PrecisionMaxSpeed),  
            !robotCentricSup.getAsBoolean(), 
            true, targetAngle
        );
            
        }
        /*Default two joystick teleop drive */
        else{
           
            s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.PrecisionMaxSpeed), 
            rotationVal * Constants.Swerve.PrecisionMaxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
            );
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
