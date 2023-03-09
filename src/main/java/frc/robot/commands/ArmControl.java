// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

public class ArmControl extends CommandBase {
    private final Elevator el;
    /** Creates a new ArmControl. */
    public ArmControl(Elevator ele) {
        // Use addRequirements() here to declare subsystem dependencies.
        el = ele;
        addRequirements(ele);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.stick_control.getRawButtonPressed(4)) {
            el.work = !el.work;
            el.up = 1;
            if (!el.work) {
                el.last_pos = el.pos;
            }
        } else if (RobotContainer.stick_control.getRawButtonPressed(1)) {
            el.work = !el.work;
            el.up = -1;
            if (!el.work) {
                el.last_pos = el.pos;
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
