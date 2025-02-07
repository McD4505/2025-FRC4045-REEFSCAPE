// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

  public static Pose2d transformFromTag(int id, Transform2d transform) {
    Pose2d tagPose = getTargetPose(id);
    return tagPose.plus(transform);
  }

  public static int getCurrentTagId(String limelightName) {
    return (int) LimelightHelpers.getFiducialID(limelightName);
  }

  public static Pose2d getTargetPose(int id) {
    if(id <= 0) return new Pose2d();

    return fieldLayout.getTagPose(id).get().toPose2d();
  }

  public static void targetBranch(CommandSwerveDrivetrain drivetrain, boolean isLeft) {
    int tagId = Vision.getCurrentTagId("limelight-two");

    int sign = isLeft ? -1 : 1;
    Translation2d baseTranslation = new Translation2d(0.5, Units.inchesToMeters(sign * 6));

    Transform2d transform = new Transform2d(baseTranslation, Rotation2d.fromDegrees(180));

    Pose2d targetPose = Vision.transformFromTag(tagId, transform);
    drivetrain.setTargetPose(targetPose);
  }

  public static void targetStation(CommandSwerveDrivetrain drivetrain) {
    int tagId = Vision.getCurrentTagId("limelight");

    Translation2d baseTranslation = new Translation2d(0.5, 0);

    Transform2d transform = new Transform2d(baseTranslation, Rotation2d.fromDegrees(180));

    Pose2d targetPose = Vision.transformFromTag(tagId, transform);
    drivetrain.setTargetPose(targetPose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

