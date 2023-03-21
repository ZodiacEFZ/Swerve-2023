// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class auto extends CommandBase {
  private final SwerveSubsystem swerve_subsystem;
  public double[] angleGoal = new double[8], velocityGoal = new double[8];
  public double angle;
  public double error, errorSum;
  private double kP, kI, kD;
  private boolean flag;
  /** Creates a new auto. */
  public auto(SwerveSubsystem s1) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve_subsystem = s1;
    addRequirements(s1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle=swerve_subsystem.get_angle();
    if(Math.abs(angle)<0.5){
        if (!flag) swerve_subsystem.car_oriented(0, 0.5, 0);
        else swerve_subsystem.car_oriented(0, 0, 0);
    } else {
        error = angle;
        swerve_subsystem.car_oriented(0, error * kP + errorSum * kI, 0);
    }
    angleGoal = swerve_subsystem.get_theta();
    velocityGoal = swerve_subsystem.get_velocity();
    for (int i = 1; i <= 4; i++) {
        angleGoal[i] = (Math.toDegrees(angleGoal[i])) % 360;
        velocityGoal[i] *= 0.2;
        velocityGoal[i] = 18000 * velocityGoal[i] + 2000;
    }

    RobotContainer.LeftFrontSwerveModule.setStatus(angleGoal[1], velocityGoal[1]);
    RobotContainer.RightFrontSwerveModule.setStatus(angleGoal[2], velocityGoal[2]);
    RobotContainer.RightBackSwerveModule.setStatus(angleGoal[3], velocityGoal[3]);
    RobotContainer.LeftBackSwerveModule.setStatus(angleGoal[4], velocityGoal[4]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
