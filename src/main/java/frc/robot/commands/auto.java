// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class auto extends CommandBase {
  private final Pneumatics pn;
  
  /** Creates a new auto. */
  public auto(Pneumatics pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    pn = pneumatics;
    addRequirements(pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // pn.stretch();
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // pn.stretcher.set(false);
    // pn.stretched = true;
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


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
// // import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.SwerveSubsystem;

// public class auto extends CommandBase {
//   private final SwerveSubsystem swerve_subsystem;
//   public double[] angleGoal = new double[8], velocityGoal = new double[8];
//   public double angle,targetangle,t;
//   private double kP=0.06;
//   public boolean flag=false;
//   public Timer timer;
//   /** Creates a new auto. */
//   public auto(SwerveSubsystem s1) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     swerve_subsystem = s1;
//     addRequirements(s1);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     targetangle = swerve_subsystem.get_field_angle();
//     timer.reset();
//     t=timer.get();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     angle=swerve_subsystem.get_angle();
//     SmartDashboard.putNumber("angleeee", angle);
//     double Error = targetangle - swerve_subsystem.get_field_angle();
//     if (Error > 180)
//       Error -= 360;
//     else if (Error < -180)
//       Error += 360;
//     // rot_value = error * (0.5 - kv / 3);
//     double rot_value = Error * 0.1;
//     if(Math.abs(angle)>3||timer.get()-t>5) flag=true;// has reached the charge station
//     if(!flag){
//         swerve_subsystem.car_oriented(0, 0.2, rot_value);
//     } else {
//         double error = -angle,v=error * kP;
//         if(Math.abs(v)>0.2){
//           if(v>0) v=0.2;
//           else v=-0.2;
//         } 
//         swerve_subsystem.car_oriented(0, v , 0);
//     }
//     angleGoal = swerve_subsystem.get_theta();
//     velocityGoal = swerve_subsystem.get_velocity();
//     for (int i = 1; i <= 4; i++) {
//         angleGoal[i] = (Math.toDegrees(angleGoal[i])) % 360;
//         velocityGoal[i] *= 0.2;
//         velocityGoal[i] = 18000 * velocityGoal[i] + 2000;
//     }

//     RobotContainer.LeftFrontSwerveModule.setStatus(angleGoal[1], velocityGoal[1]);
//     RobotContainer.RightFrontSwerveModule.setStatus(angleGoal[2], velocityGoal[2]);
//     RobotContainer.RightBackSwerveModule.setStatus(angleGoal[3], velocityGoal[3]);
//     RobotContainer.LeftBackSwerveModule.setStatus(angleGoal[4], velocityGoal[4]);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
