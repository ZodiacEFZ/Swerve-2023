// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class AprilTagAuto extends CommandBase {
    private final SwerveSubsystem swerve_Subsystem;
    private final Limelight lmcam;

    /** Creates a new AprilTagAuto. */
    public AprilTagAuto(SwerveSubsystem swerve_sub, Limelight lm) {
        // Use addRequirements() here to declare subsystem dependencies.
        lmcam = lm;
        swerve_Subsystem = swerve_sub;
        addRequirements(swerve_sub, lm);
    }

    public int err = 15;

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (SwerveDriveJoystick.getInstance().aprilTagAutoTracking) {
            if (Limelight.getInstance().angle <= err) {
                SwerveDriveJoystick.getInstance().aprilTagAutoTracking = false;
            } else {

            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
