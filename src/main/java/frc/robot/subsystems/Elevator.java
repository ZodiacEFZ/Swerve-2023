// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    /** Creates a new Elevator. */
    public Elevator() {
        work = false;
        liftSpeed = 0.2;
        up = 1;
    }

    Encoder encoder = new Encoder(0, 1);
    WPI_TalonSRX motor_front = new WPI_TalonSRX(28);
    WPI_TalonSRX motor_back = new WPI_TalonSRX(30);

    public double pos;
    public double err = 100;
    public double last_pos;

    public void stopLifting() {
        motor_front.set(0);
        motor_back.set(0);
    }

    public double liftSpeed = 0.2;
    public int up = 1;
    public boolean work = false;

    private boolean change = false;

    public void lift() {
        motor_front.set(up * liftSpeed);
        motor_back.set(up * liftSpeed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        pos = encoder.getDistance();
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
        if (work || Math.abs(pos - last_pos) <= err) {
            change = false;
        }
    }
}
