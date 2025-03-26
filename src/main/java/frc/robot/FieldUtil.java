// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */

public class FieldUtil {


    public static Transform2d getScoringTransform(boolean isLeft) {
        int sign = isLeft ? -1 : 1;
        double offset = isLeft ? 7 : 6;
        Translation2d baseTranslation = new Translation2d(0.50, Units.inchesToMeters(sign * offset + 12));

        return new Transform2d(baseTranslation, Rotation2d.fromDegrees(180));
    }

    public static Transform2d getStationTransform() {
        Translation2d baseTranslation = new Translation2d(0.50, -0.2);

        return new Transform2d(baseTranslation, Rotation2d.fromDegrees(0));
    }

    public static Transform2d getStationTransformRight() {
        Translation2d baseTranslation = new Translation2d(0.48, 0.5);

        return new Transform2d(baseTranslation, Rotation2d.fromDegrees(0));
    }

    public static Transform2d getStationTransformLeft() {
        Translation2d baseTranslation = new Translation2d(0.48, -0.8);

        return new Transform2d(baseTranslation, Rotation2d.fromDegrees(0));
    }

    public static Transform2d getPreScoringTransform() {
        Translation2d baseTranslation = new Translation2d(1, 0);

        return new Transform2d(baseTranslation, Rotation2d.fromDegrees(180));
    }
}
