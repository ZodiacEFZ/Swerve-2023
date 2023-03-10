// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AprilTagAuto;
import frc.robot.commands.ArmControl;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveDriveJoystick;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {

    public static Joystick stick = new Joystick(0);
    public static Joystick stick_control = new Joystick(1);
    // motor command and subsystem
    // public static intake m_intake = new intake(Constants.intake_port);
    // public static arm Arm = new arm(Constants.arm_port);
    // public static SignalControl m_signalcontrol = new
    // SignalControl(/*m_intake,Arm*/);
    public static SwerveSubsystem m_calculate = new SwerveSubsystem();
    /*
     * leftBack: 12
     * leftFront: 18
     * rightBack: 14
     * rightFront: 16
     * liftingFront: 28
     * 
     */
    public static SwerveModule LeftFrontSwerveModule = new SwerveModule(1, 34, 18, 1818, 0);
    public static SwerveModule LeftBackSwerveModule = new SwerveModule(4, 36, 12, 4787, 0);
    public static SwerveModule RightFrontSwerveModule = new SwerveModule(2, 32, 16, 2564, 1);
    public static SwerveModule RightBackSwerveModule = new SwerveModule(3, 38, 14, 2666, 1);
    public static Limelight Limelight = new Limelight();
    public static Elevator elevator = new Elevator();
    public static SwerveDriveJoystick m_swerve_drive = new SwerveDriveJoystick(m_calculate, Limelight);
    public static AprilTagAuto AprilTagTracking = new AprilTagAuto(m_calculate, Limelight);
    public static ArmControl armControl = new ArmControl(elevator);
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(m_exampleSubsystem::exampleCondition)
                .onTrue(new ExampleCommand(m_exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is
        // pressed,
        // cancelling on release.
        m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Autos.exampleAuto(m_exampleSubsystem);
    }
}
