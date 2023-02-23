// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Swerve_drive_joystick;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Swerve_subsystem;
import frc.robot.subsystems.limelight;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

 
public class RobotContainer {

  public static Joystick stick = new Joystick(0);
  //motor command and subsystem
  // public static intake m_intake = new intake(Constants.intake_port);
  // public static arm Arm = new arm(Constants.arm_port);
 // public static SignalControl m_signalcontrol = new SignalControl(/*m_intake,Arm*/);
  public static Swerve_subsystem m_calculate = new Swerve_subsystem(); 
  public static SwerveModule LeftFrontSwerveModule = new SwerveModule(1, 34, 14, 3601, 0);      //(number, angleMotorPort, velocityMotorPort, zeroPosition)
  public static SwerveModule RightFrontSwerveModule = new SwerveModule(2, 32, 12, 3831, 1);
  public static SwerveModule RightBackSwerveModule = new SwerveModule(3, 38, 18, 754, 1);
  public static SwerveModule LeftBackSwerveModule = new SwerveModule(4, 36, 16, 3491, 0);
  public static limelight Limelight = new limelight();
  public static Swerve_drive_joystick m_swerve_drive = new Swerve_drive_joystick(m_calculate,Limelight);
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
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
