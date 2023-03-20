// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Tongs extends SubsystemBase {
    /** Creates a new pair of Tongs. */
    public Tongs() {
        
        flipingLeftMotor = new WPI_TalonFX(Constants.flipingLeftMotorID);
        flipingRightMotor = new WPI_TalonFX(Constants.flipingRightMotorID);
        flipingLeftMotor.configFactoryDefault();
        flipingLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        flipingLeftMotor.config_kP(0, 0.1); // *PID still need improvement
        flipingLeftMotor.config_kI(0, 0);
        flipingLeftMotor.config_kD(0, 0);
        flipingLeftMotor.setNeutralMode(NeutralMode.Coast);
        flipingLeftMotor.setSensorPhase(true);
        if (inverse) {
            flipingLeftMotor.setInverted(true);
        } else {
            flipingLeftMotor.setInverted(false);
        }
        flipingRightMotor.follow(flipingLeftMotor);
        flipingRightMotor.setInverted(InvertType.FollowMaster);
        lpos = flipingLeftMotor.getSelectedSensorPosition();
    }

    public WPI_TalonFX flipingLeftMotor;
    public WPI_TalonFX flipingRightMotor;
    
    public boolean inverse = false;
    public double pos, lpos;
    public double min = 0;
    public double max = 0;
    public boolean isFliping = false;
    public boolean up = false;

    public void setStill() {
        // flipingLeftMotor.set(0);
        flipingLeftMotor.set(0.05);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        pos = flipingLeftMotor.getSelectedSensorPosition();
        if (isFliping && up) {
            flipingLeftMotor.set(0.1);
        } else if (isFliping && !up) {
            flipingLeftMotor.set(-0.1);
        } else {
            // setStill();
            // pid.setTolerance(50);
            // flipingLeftMotor.set(pid.calculate(pos, lpos));
            flipingLeftMotor.set(ControlMode.Position, lpos);
        }
    }
}