// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Tongs;

public class ArmControl extends CommandBase {
    private final Elevator el;
    private final Tongs to;
    private final Pneumatics pn;

    public int valuePOV_x, valuePOV_y;

    public boolean takingIn = false;

    /** Creates a new ArmControl. */
    public ArmControl(Elevator ele, Tongs tong, Pneumatics pneumatics) {
        // Use addRequirements() here to declare subsystem dependencies.
        el = ele;
        to = tong;
        pn = pneumatics;
        addRequirements(ele, tong, pneumatics);
    }

    public ArmControl(Elevator ele, Tongs tongs) {
        // Use addRequirements() here to declare subsystem dependencies.
        el = ele;
        to = tongs;
        pn = null;
        addRequirements(ele, tongs);
    }

    public ArmControl(Tongs tongs, Pneumatics pneumatics) {
        el = null;
        to = tongs;
        pn = pneumatics;
        addRequirements(tongs, pneumatics);
    }

    public ArmControl(Tongs tongs) {
        el = null;
        to = tongs;
        pn = null;
        addRequirements(tongs);
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
            // pn.stretch();
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
        if (RobotContainer.stick.getRawButtonPressed(3)) {
            System.out.println("scratchPressed");
            pn.scratch();
        }
        if (RobotContainer.stick.getRawButtonPressed(4)) {
            System.out.println("stretchPressed");
            pn.stretch();
        }
        if (RobotContainer.stick.getRawButtonPressed(2)) {
            takingIn = !takingIn;
            if (takingIn) to.takein();
            else to.takeinStop();
        }
        // lifting motor encoder value
        SmartDashboard.putNumber("liftingEncoderValue", to.liftingBackMotor.getSelectedSensorPosition());
        to.elePos4Test = to.liftingBackMotor.getSelectedSensorPosition();
        if (RobotContainer.stick_control.getRawButtonPressed(1)) {
            to.eleMode = 0;
        } else if (RobotContainer.stick_control.getRawButtonPressed(2)) {
            to.eleMode = 1;
        } else if (RobotContainer.stick_control.getRawButtonPressed(3)) {
            to.eleMode = 2;
        } else if (RobotContainer.stick_control.getRawButtonPressed(4)) {
            to.eleMode = 3;
        }
        if (to.eleMode == 0) {
            // to.liftingBackMotor.set(ControlMode.Position, to.elePosGoal[0]);
            if (to.elePos4Test < 0) {
                // to.liftingBackMotor.set(ControlMode.PercentOutput, 0);
            }
        } else if (to.eleMode == 1) { // slide port
            // to.liftingBackMotor.set(ControlMode.Position, to.elePosGoal[1]);
            if (!pn.stretched) {
                pn.stretch();
            }
        } else if (to.eleMode == 2) {
            // to.liftingBackMotor.set(ControlMode.Position, to.elePosGoal[2]);
        } else if (to.eleMode == 3) {
            // to.liftingBackMotor.set(ControlMode.Position, to.elePosGoal[3]);
        }
        if (RobotContainer.stick.getPOV() == -1) {
            if (RobotContainer.stick_control.getPOV() == 0) {
                valuePOV_x = 1;
            } else if (RobotContainer.stick_control.getPOV() == 180) {
                valuePOV_x = -1;
            } else if (RobotContainer.stick_control.getPOV() == 90) {
                valuePOV_y = 1;
            } else if (RobotContainer.stick_control.getPOV() == 270) {
                valuePOV_y = -1;
            } else {
                valuePOV_x = valuePOV_y = 0;
            }
        } else {
            if (RobotContainer.stick.getPOV() == 0 || RobotContainer.stick.getPOV() == 180) {
                if (RobotContainer.stick.getPOV() == 0) valuePOV_y = 1;
                else if (RobotContainer.stick.getPOV() == 180) valuePOV_y = -1;
                if (RobotContainer.stick_control.getPOV() == 90) valuePOV_x = 1;
                else if (RobotContainer.stick_control.getPOV() == 270) valuePOV_x = -1;
                else valuePOV_x = 0;
            }
            if (RobotContainer.stick.getPOV() == 90 || RobotContainer.stick.getPOV() == 270) {
                if (RobotContainer.stick.getPOV() == 90) valuePOV_x = 1;
                else if (RobotContainer.stick.getPOV() == 270) valuePOV_x = -1;
                if (RobotContainer.stick_control.getPOV() == 0) valuePOV_y = 1;
                else if (RobotContainer.stick_control.getPOV() == 180) valuePOV_y = -1;
                else valuePOV_y = 0;
            }
        }
        if (valuePOV_x == 1) {
            to.takein();
        } else if (valuePOV_x == -1) {
            to.putout();
        } else {
            to.takeinStop();
        }
        if (valuePOV_y == 1) {
            pn.stretch();
        } else if (valuePOV_y == -1) {
            pn.scratch();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        to.flag = false;
        to.lpos = to.pos;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
