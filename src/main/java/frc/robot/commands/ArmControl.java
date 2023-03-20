// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Tongs;

public class ArmControl extends CommandBase {
    private final Elevator el;
    private final Tongs to;
    private final Pneumatics ph;

    /** Creates a new ArmControl. */
    public ArmControl(Elevator ele, Tongs tong, Pneumatics pneumatics) {
        // Use addRequirements() here to declare subsystem dependencies.
        el = ele;
        to = tong;
        ph = pneumatics;
        addRequirements(ele, tong, pneumatics);
    }

    public ArmControl(Elevator ele, Tongs tongs) {
        // Use addRequirements() here to declare subsystem dependencies.
        el = ele;
        to = tongs;
        ph = null;
        addRequirements(ele, tongs);
    }

    public ArmControl() {
        el = null;
        to = null;
        ph = null;
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
        // if (RobotContainer.stick_control.getRawButtonPressed(2)) {
            // ph.stretch();
        // }
        if (RobotContainer.stick_control.getRawButtonPressed(2)) {
            if (to.up) {
                to.isFliping = !to.isFliping;
            } else {
                to.up = true;
                if (!to.isFliping) to.isFliping = true;
            }
            if (!to.isFliping) to.lpos = to.pos;
        }
        if (RobotContainer.stick_control.getRawButtonPressed(3)) {
            if (to.up) {
                to.up = false;
                if (!to.isFliping) to.isFliping = true;
            } else {
                to.isFliping = !to.isFliping;
            }
            if (!to.isFliping) to.lpos = to.pos;
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
