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
import frc.robot.FieldUtil;
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

  public static Pose2d getScoringPose(int tagId, boolean isLeft) {
    Transform2d transform = FieldUtil.getScoringTransform(isLeft);
    return transformFromTag(tagId, transform);
  }

  public static Pose2d getStationPose(int tagId) {
    Transform2d transform = FieldUtil.getStationTransform();
    return transformFromTag(tagId, transform);
  }

  /**
   * Transforms the apriltag pose by a given transform
   * @param id apriltag id
   * @param transform transform to apply
   * @return transformed pose
   */
  public static Pose2d transformFromTag(int id, Transform2d transform) {
    Pose2d tagPose = getTagPose(id);
    return tagPose.plus(transform);
  }

  public static int getCurrentTagId(String limelightName) {
    return (int) LimelightHelpers.getFiducialID(limelightName);
  }

  /**
   * Gets the apriltag pose from the field layout
   * @param id apriltag id
   * @return apriltag pose
   */
  public static Pose2d getTagPose(int id) {
    if(id <= 0) return new Pose2d();  // if invalid id, return origin

    return fieldLayout.getTagPose(id).get().toPose2d();
  }

  /**
   * Sets the target pose of the drivetrain to the target branch 
   * based on currently detected reef apriltag
   * @param drivetrain drivetrain
   * @param isLeft true if left branch, false if right branch
   */
  public static void targetBranch(CommandSwerveDrivetrain drivetrain, boolean isLeft) {
    int tagId = Vision.getCurrentTagId("limelight-two");

    if(tagId <= 0) return;
    drivetrain.setTargetPose(getScoringPose(tagId, isLeft));
  }

  public static void targetBranchFromId(CommandSwerveDrivetrain drivetrain, int tagId, boolean isLeft) {
    drivetrain.setTargetPose(getScoringPose(tagId, isLeft));
  }

  public static void targetStationFromId(CommandSwerveDrivetrain drivetrain, int tagId) {
    drivetrain.setTargetPose(getStationPose(tagId));
  }

  /**
   * Sets the target pose of the drivetrain to the target station 
   * based on currently detected reef apriltag
   * @param drivetrain drivetrain
   */
  public static void targetStation(CommandSwerveDrivetrain drivetrain) {
    int tagId = Vision.getCurrentTagId("limelight");

    if(tagId <= 0) return;
    drivetrain.setTargetPose(getStationPose(tagId));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

