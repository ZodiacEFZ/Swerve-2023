// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
    /** Creates a new Pneumatics. */
    public Pneumatics() {
        pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        scratcher = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        stretcher = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
        pcmCompressor.enableDigital();
        scratcher.set(true);
        stretcher.set(true);
    }

    public Compressor pcmCompressor;
    public Solenoid scratcher, stretcher;

    public void scratch() { //抓取
        scratcher.toggle();
    }

    public void stretch() { //翻转
        stretcher.toggle();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
    }
}
