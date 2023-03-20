// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    /** Creates a new Elevator. */
    public Elevator() {
        motor_front = new WPI_TalonSRX(Constants.liftingMotorFrontID);
        motor_back = new WPI_TalonSRX(Constants.liftingMotorBackID);
        motor_front.configFactoryDefault();
        motor_back.configFactoryDefault();
        // motor_front.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        motor_back.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        motor_back.config_kP(0, 0.05);
        motor_back.config_kI(0, 0);
        motor_back.config_kD(0, 0);
        motor_front.follow(motor_back);
        motor_front.setInverted(InvertType.FollowMaster);
    }

    WPI_TalonSRX motor_front;
    WPI_TalonSRX motor_back;

    public double pos;
    public double err = 100;
    public double last_pos;

    public double liftSpeed = 0.2;
    public int up = 1;
    public boolean work = false;

    private boolean change = false;

    public void lift() {
        motor_front.set(up * liftSpeed);
        motor_back.set(up * liftSpeed);
    }
    
    public void stopLifting() {
        motor_front.set(0);
        motor_back.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        pos = motor_back.getSelectedSensorPosition();
        if (work) {
            lift();
        } else {
            stopLifting();
        }
        SmartDashboard.putNumber("elePos", pos);
        SmartDashboard.putNumber("eleLastPos", last_pos);
        SmartDashboard.putBoolean("remedy", change);
        while (!work && Math.abs(pos - last_pos) > err) {
            change = true;
        }
        change = false;
    }
}
