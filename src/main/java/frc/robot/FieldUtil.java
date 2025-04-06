// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Vision;

/** Add your docs here. */

public class FieldUtil {

    public static enum ReefSide {
        CLOSE, FAR, LEFT_CLOSE, LEFT_FAR, RIGHT_CLOSE, RIGHT_FAR, LEFT_STATION, RIGHT_STATION
    }

    public static int getTagId(ReefSide side, boolean isRed) {
        int id = 0;
        switch (side) {
            case CLOSE:
                id = isRed ? 7 : 18;
                break;
            case FAR:
                id = isRed ? 10 : 21;
                break;
            case LEFT_CLOSE:
                id = isRed ? 6 : 19;
                break;
            case LEFT_FAR:
                id = isRed ? 11 : 20;
                break;
            case RIGHT_CLOSE:
                id = isRed ? 8 : 17;
                break;
            case RIGHT_FAR:
                id = isRed ? 9 : 22;
                break;
            case LEFT_STATION:
                id = isRed ? 1 : 13;
                break;
            case RIGHT_STATION:
                id = isRed ? 2 : 12;
                break;
        }
        return id;
    }

    public static Pose2d getPreScoringPose(ReefSide side, boolean isRed) {
        return Vision.getPreScoringPose(getTagId(side, isRed));
    }

    public static Pose2d getStationPose(ReefSide side, boolean isRed) {
        return Vision.getStationPose(getTagId(side, isRed));
    }

    public static Transform2d getScoringTransform(boolean isLeft) {
        int sign = isLeft ? -1 : 1;
        double offset = isLeft ? 6 : 7;
        Translation2d baseTranslation = new Translation2d(0.50, Units.inchesToMeters(sign * offset + 11));

        return new Transform2d(baseTranslation, Rotation2d.fromDegrees(180));
    }

    public static Transform2d getStationTransform() {
        Translation2d baseTranslation = new Translation2d(0.50, -0.2);  // 0.5m out, 0.2m left (from tag perspective)

        return new Transform2d(baseTranslation, Rotation2d.fromDegrees(0));  // 0 degree rotation relative to tag
    }

    public static Transform2d getStationTransformRight() {
        Translation2d baseTranslation = new Translation2d(0.43, 0.5);

        return new Transform2d(baseTranslation, Rotation2d.fromDegrees(0));
    }

    public static Transform2d getStationTransformLeft() {
        Translation2d baseTranslation = new Translation2d(0.43, -0.8);

        return new Transform2d(baseTranslation, Rotation2d.fromDegrees(0));
    }

    public static Transform2d getPreScoringTransform() {
        Translation2d baseTranslation = new Translation2d(1, 0);

        return new Transform2d(baseTranslation, Rotation2d.fromDegrees(180));
    }
}
