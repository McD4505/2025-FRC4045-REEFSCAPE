// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private static AprilTagFieldLayout fieldLayout;
  
  public Vision() {
    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
    } catch(IOException e) {
      DriverStation.reportError("AprilTag field layout not found.", e.getStackTrace());
    }
  }

  public static Pose2d getTargetPose() {
    int id = (int)LimelightHelpers.getFiducialID("limelight-two");
    
    if(id <= 0) return new Pose2d();

    SmartDashboard.putNumber("detected id", id);
    return fieldLayout.getTagPose(id).get().toPose2d();
    // return AprilTagPoses.get((int)LimelightHelpers.getFiducialID("limelight-two"));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

