package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier cardFront;
    private BooleanSupplier cardBack;
    private BooleanSupplier cardLeft;
    private BooleanSupplier cardRight;

    public TeleopSwerve(
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
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.cardFront = cardFront;
        this.cardBack = cardBack;
        this.cardLeft = cardLeft;
        this.cardRight = cardRight;

    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        double targetAngle=0;

        /* Drive */
        if (cardFront.getAsBoolean()==true || cardBack.getAsBoolean()==true || cardLeft.getAsBoolean()==true || cardRight.getAsBoolean()==true) {
            if (cardFront.getAsBoolean()==true){
                targetAngle=0;
            }
            else if (cardBack.getAsBoolean()==true){
                targetAngle=180;
            }
            else if (cardLeft.getAsBoolean()==true){
                targetAngle=90;
            }
            else if (cardRight.getAsBoolean()==true){
                targetAngle=270;
            }

            s_Swerve.lockDrive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                !robotCentricSup.getAsBoolean(),
                true,
                targetAngle
            );
        } else {
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                !robotCentricSup.getAsBoolean(),
                true
            );
        }
    }
}