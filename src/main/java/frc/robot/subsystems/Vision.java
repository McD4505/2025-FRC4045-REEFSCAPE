// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
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

  public static Pose2d getStationPoseRight(int id) {
    return transformFromTag(id, FieldUtil.getStationTransformRight());
  }

  public static Pose2d getStationPoseLeft(int id) {
    return transformFromTag(id, FieldUtil.getStationTransformLeft());
  }

  public static void addVisionMeasurementMT1(CommandSwerveDrivetrain drivetrain, String limelightName) {
    var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    if (mt1 != null && mt1.tagCount > 0) {
      double distance = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getTranslation().getNorm();
      double stdDev = Math.min(distance/2, 3.0) / mt1.tagCount;

      drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(stdDev, stdDev, stdDev * 5));
      drivetrain.addVisionMeasurement(mt1.pose, Utils.fpgaToCurrentTime(mt1.timestampSeconds));
    }
  }

  public static void addVisionMeasurementMT2(CommandSwerveDrivetrain drivetrain, String limelightName, double omegaRps) {
    var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (mt2 != null && mt2.tagCount > 0 && omegaRps < 2.0) {
      double distance = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getTranslation().getNorm();
      double stdDev = Math.min(distance/4, 3.0) / mt2.tagCount;

      drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(stdDev, stdDev, 999999));
      drivetrain.addVisionMeasurement(mt2.pose, Utils.fpgaToCurrentTime(mt2.timestampSeconds));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

