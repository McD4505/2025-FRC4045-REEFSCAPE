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
        double offset = isLeft ? 6 : 6;
        Translation2d baseTranslation = new Translation2d(0.48, Units.inchesToMeters(sign * offset + 12));

        return new Transform2d(baseTranslation, Rotation2d.fromDegrees(180));
    }

    public static Transform2d getStationTransform() {
        Translation2d baseTranslation = new Translation2d(0.48, -0.2);

        return new Transform2d(baseTranslation, Rotation2d.fromDegrees(0));
    }
}
