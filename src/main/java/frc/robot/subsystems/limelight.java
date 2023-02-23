// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limelight extends SubsystemBase {
  /** Creates a new limelight. */
  public limelight() {
    LimelightHelpers.profileJSON = false;
    limelightConfig.getEntry("camMode").setNumber(0);
  }
  NetworkTable limelightConfig = NetworkTableInstance.getDefault().getTable("limelight");

  public double data[];

  public double x, y, angle;
  public double tid;

  public boolean trackingStatus;

  public String JSON = "";

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    trackingStatus = (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1 ? true
        : false);

    JSON = LimelightHelpers.getJSONDump("limelight"); //Get the JSON string

    try {
      LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");

      /*
       * this part of the code use provided method from the LimelightHelpers.java
       * if you have to update this, remember to make the corresponding object public in the code
       * or we may contribute to the source code on the github, under this condition you can ignore this step
       */

      LimelightHelpers.LimelightTarget_Fiducial[] dataFiducial = llresults.targetingResults.targets_Fiducials;

      data = dataFiducial[0].cameraPose_TargetSpace;
    } catch (Exception e) { // using "try...catch..." to avoid tracking the null pointer (this will cause startCompetition() error)
      // System.out.println(e.toString());
      data = new double[6];
      // credit: m5; mol!!!!
    }

    x = data[1]; //get detailed value from the former object
    y = data[2];
    angle = data[4];

    tid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightAngle", angle);
    SmartDashboard.putNumber("LimelightTagId", tid);
    SmartDashboard.putBoolean("LimelightTracking", trackingStatus);

    SmartDashboard.putString("LimelightJSON", JSON); //print out the JSON string
  }
}
