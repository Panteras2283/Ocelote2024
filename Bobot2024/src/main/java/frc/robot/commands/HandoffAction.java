// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class HandoffAction extends Command {
  private Feeder Feeder;
  private Shooter Shooter;
  /** Creates a new HandoffAction. */
  public HandoffAction(Feeder Feeder, Shooter Shooter) {
    this.Feeder = Feeder;
    this.Shooter = Shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Feeder, Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Feeder.handoffPosition();
    Shooter.setPosition(Constants.ShooterConstants.handoffPos);
    Shooter.receive();
    if(Shooter.shareEncoder()<= Constants.ShooterConstants.handoffPos+0.1){
      Feeder.passThrough();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("HANDOFF ENDED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Shooter.getSensor() == false) {
      return true;
    } else {
      return false;
    }
  }
}
