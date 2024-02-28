package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    private final Joystick station = new Joystick(2);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kBack.value); //GYRO SET TO ZERO
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kStart.value); //ROBOT-ORIENTED
    private final JoystickButton cardinalFront = new JoystickButton(driver, XboxController.Button.kY.value); //CARD. FRONT POS
    private final JoystickButton cardinalBack = new JoystickButton(driver, XboxController.Button.kA.value); //CARD BACK POS
    private final JoystickButton cardinalLeft = new JoystickButton(driver, XboxController.Button.kX.value); //CARD LEFT POS
    private final JoystickButton cardinalRight = new JoystickButton(driver, XboxController.Button.kB.value); //CARD RIGHT POS
    private final JoystickButton precisionMode = new JoystickButton(driver, XboxController.Button.kRightBumper.value); //PRECISION MODE
    private final JoystickButton dropFeeder = new JoystickButton(operator, XboxController.Button.kY.value); //COMER ARO DEL SUELO
    private final JoystickButton amp = new JoystickButton(operator, XboxController.Button.kA.value); //PONER FEEDER EN AMP Y ESCUPIR
    private final JoystickButton handoff = new JoystickButton(operator, XboxController.Button.kX.value); //PONER FEEDER EN HANDOFF Y SACAR
    private final JoystickButton speakerShooter = new JoystickButton(operator, XboxController.Button.kB.value); //PONER SHOOTER EN POS PARA SPEAKER
    private final JoystickButton ATLocker = new JoystickButton(station, 9); //BLOQUEAR LA LIMELIGHT
    private final JoystickButton primeButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value); //DISPARAR AL SPEAKER
    private final JoystickButton soltar = new JoystickButton(operator, XboxController.Button.kRightBumper.value); //SOLTAR ARO
    //private final JoystickButton LEDboost = new JoystickButton(operator, XboxController.Button.kB.value);


    /*Limelight Buttons */
    private final JoystickButton speakerTarget = new JoystickButton(driver, XboxController.Button.kLeftStick.value); //ACTIVATES AprilT DET.    
    private final JoystickButton ampTarget = new JoystickButton(driver, XboxController.Button.kRightStick.value); //ACTIVATES AprilT DET.


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Feeder f_Feeder = new Feeder();
    private final Shooter s_Shooter = new Shooter();
    private final Limelight l_Limelight = new Limelight();
    //private final LD l_LD = new LD();

    /*Sensors */


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> cardinalFront.getAsBoolean(),
                () -> cardinalBack.getAsBoolean(),
                () -> cardinalLeft.getAsBoolean(),
                () -> cardinalRight.getAsBoolean()

            )
        );

        f_Feeder.setDefaultCommand(new FeederDefault(
            f_Feeder
        ));

        s_Shooter.setDefaultCommand(new ShooterDefault(
            s_Shooter
        ));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        dropFeeder.onTrue(new FeederIntakeNote(f_Feeder));
        dropFeeder.onFalse(f_Feeder.getDefaultCommand());

        handoff.onTrue(new HandoffAction(f_Feeder, s_Shooter));
        handoff.onFalse(f_Feeder.getDefaultCommand());
        handoff.onFalse(s_Shooter.getDefaultCommand());
        
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        //LEDboost.onTrue(new InstantCommand(() -> l_LD.mode3()));
        //LEDboost.onFalse(new InstantCommand(()-> l_LD.idle()));

        primeButton.onTrue(new ShooterAim(s_Shooter, l_Limelight));
        primeButton.onFalse(s_Shooter.getDefaultCommand());

        amp.onTrue(new ShooterAMP(s_Shooter));
        amp.onFalse(s_Shooter.getDefaultCommand());

        soltar.onTrue(new InstantCommand(() -> s_Shooter.soltar()));
        soltar.onFalse(new InstantCommand(() -> s_Shooter.stopRetainer()));

        primeButton.onTrue(
            new LimelightTurretSwerve(
                l_Limelight,
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        primeButton.onFalse(
            s_Swerve.getDefaultCommand()
        );

        precisionMode.onTrue(
            new PrecisionSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> cardinalFront.getAsBoolean(),
                () -> cardinalBack.getAsBoolean(),
                () -> cardinalLeft.getAsBoolean(),
                () -> cardinalRight.getAsBoolean()
            )
        );

        precisionMode.onFalse(
            s_Swerve.getDefaultCommand()
        );

       

        /*ampTarget.onTrue(
            new SideSwerve(
                l_Limelight,
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        ampTarget.onFalse(
            s_Swerve.getDefaultCommand()
        );*/

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new PathPlannerAuto("AUTO2");
    }
}
