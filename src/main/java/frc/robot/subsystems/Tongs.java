// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Tongs extends SubsystemBase {
    /** Creates a new pair of Tongs. */
    public Tongs() {
        // init config of flipping motor
        armMotorLeft.configFactoryDefault();
        armMotorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        armMotorRight.follow(armMotorLeft);
        armMotorRight.setInverted(InvertType.OpposeMaster);
        armMotorLeft.config_kP(0, 0.02); // PID may be improved
        armMotorLeft.config_kI(0, 0);
        armMotorLeft.config_kD(0, 0.05);
        armMotorLeft.configPeakOutputForward(0.4); // caution value
        armMotorLeft.configPeakOutputReverse(-0.4);
        armMotorLeft.setNeutralMode(NeutralMode.Brake);
        armMotorLeft.setSensorPhase(true);
        // init config of elevator motor
        liftingBackMotor.configFactoryDefault();
        liftingBackMotor.setInverted(true);
        liftingBackMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,
                0, 0);
        liftingFrontMotor.follow(liftingBackMotor);
        liftingFrontMotor.setInverted(InvertType.FollowMaster);
        liftingBackMotor.config_kP(0, 0.2); // PID should be improved
        liftingBackMotor.config_kI(0, 0);
        liftingBackMotor.config_kD(0, 0);
        liftingBackMotor.configPeakOutputForward(0.4);
        liftingBackMotor.configPeakOutputReverse(-0.4);
        liftingBackMotor.setNeutralMode(NeutralMode.Brake);
        liftingBackMotor.setSensorPhase(true);
        // init config of pneumatics controller

    }

    public WPI_TalonFX armMotorLeft = new WPI_TalonFX(20);
    public WPI_TalonFX armMotorRight = new WPI_TalonFX(6);

    public WPI_TalonSRX liftingFrontMotor = new WPI_TalonSRX(30);
    public WPI_TalonSRX liftingBackMotor = new WPI_TalonSRX(28);

    public WPI_TalonSRX takeinMotor = new WPI_TalonSRX(8);

    public double armSpeed;
    public double pos, lpos, posEle, lposEle;
    public double elePos4Test;
    public boolean isFliping = false;
    public boolean isLifting = false;
    public boolean up = false;
    public boolean upEle = true;
    public boolean flag = false;
    public int eleMode = 0;
    public double[] elePosGoal = { 1, 2, 3, 4 };
    public double armUpControlValue = 0, armDownControlValue = 0;
    public int valuePOV_x, valuePOV_y;

    public void takein() {
        takeinMotor.set(-0.6);
        System.out.println("takein");
    }

    public void putout() {
        takeinMotor.set(0.6);
        System.out.println("putout");
    }

    public void takeinStop() {
        takeinMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("armPos", pos);
        SmartDashboard.putNumber("armLastPos", lpos);
        lpos = pos;
        // if (RobotContainer.stick.getRawAxis(2) < 0.1 && RobotContainer.stick.getRawAxis(3) < 0.1) {
        //     armUpControlValue = RobotContainer.stick_control.getRawAxis(3);
        //     armDownControlValue = RobotContainer.stick_control.getRawAxis(2);
        // } else {
            armUpControlValue = RobotContainer.stick.getRawAxis(3);
            armDownControlValue = RobotContainer.stick.getRawAxis(2);
        // }
        if (armDownControlValue > 0.1) {
            System.out.println("down");
            flag = true;
            up = false;
            isFliping = true;
        } else if (armUpControlValue > 0.1) {
            System.out.println("up");
            flag = true;
            up = true;
            isFliping = true;
        } else {
            armMotorLeft.set(ControlMode.PercentOutput, 0);
            isFliping = false;
            lpos = pos;
        }
        if (armDownControlValue > 0.1 && armUpControlValue > 0.1) {
            armMotorLeft.set(ControlMode.PercentOutput, 0);
            isFliping = false;
        }
        SmartDashboard.putBoolean("isFliping", isFliping);
        if (isFliping && up) {
            System.out.println("armUp");
            armMotorLeft.set(ControlMode.PercentOutput, 0.2);
            pos = armMotorLeft.getSelectedSensorPosition();
        } else if (isFliping && !up) {
            System.out.println("armDown");
            armMotorLeft.set(ControlMode.PercentOutput, -0.1);
            pos = armMotorLeft.getSelectedSensorPosition();
        } else {
            if (flag)
                armMotorLeft.set(ControlMode.Position, lpos);
                // System.out.println("remedyArm");
            else
                lpos = pos;
        }
        // armMotorLeft.set(-0.1);
        if (valuePOV_y == 1) {
        // if (RobotContainer.stick_control.getRawAxis(5) < -0.1) {
            flag = true;
            // System.out.println("eleUp");
            upEle = true;
            isLifting = true;
        // } else if (RobotContainer.stick_control.getRawAxis(5) > 0.1) {
        } else if (valuePOV_y == -1) {
            flag = true;
            // System.out.println("eleDown");
            upEle = false;
            isLifting = true;
        } else {
            liftingBackMotor.set(ControlMode.PercentOutput, 0);
            isLifting = false;
            lposEle = posEle;
        }
        if (valuePOV_x == -1) {
            takein();
            System.out.println("takeindetected");
        } else if (valuePOV_x == 1) {
            putout();
            System.out.println("putoutdetected");
        } else {
            takeinStop();
        }
        SmartDashboard.putBoolean("isFliping", isFliping);
        if (isLifting && upEle) {
            System.out.println("eleUp");
            liftingBackMotor.set(ControlMode.PercentOutput, 0.3);
            posEle = liftingBackMotor.getSelectedSensorPosition();
        } else if (isLifting && !upEle) {
            System.out.println("eleDown");
            liftingBackMotor.set(ControlMode.PercentOutput, -0.2);
            posEle = liftingBackMotor.getSelectedSensorPosition();
        } else {
            if (flag)
                liftingBackMotor.set(ControlMode.Position, lposEle);
                // System.out.println("remedyEle");
            else
                lposEle = posEle;
        }
    }
}