// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldUtil;
import frc.robot.FieldUtil.ReefSide;
import frc.robot.commands.DriveToTargetPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Dispenser;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Elevator.ReefLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BargeWingAuto extends SequentialCommandGroup {

  public BargeWingAuto(CommandSwerveDrivetrain drivetrain, Elevator elevator, Dispenser dispenser, boolean isRed, boolean isOtherBarge) {

    int idTarget1 = FieldUtil.getTagId(isOtherBarge ? ReefSide.RIGHT_FAR : ReefSide.LEFT_FAR, isRed);
    int idTarget2 = FieldUtil.getTagId(isOtherBarge ? ReefSide.RIGHT_CLOSE : ReefSide.LEFT_CLOSE, isRed);
    int idStation = FieldUtil.getTagId(isOtherBarge ? ReefSide.RIGHT_STATION : ReefSide.LEFT_STATION, isRed);

    Pose2d nearScoringPoseLeft = Vision.getScoringPose(idTarget1, true);
    Pose2d nearStationScoringPoseLeft = Vision.getScoringPose(idTarget2, true);
    Pose2d nearStationScoringPoseRight = Vision.getScoringPose(idTarget2, false);
    Pose2d stationPose = isOtherBarge ? Vision.getStationPoseLeft(idStation) : Vision.getStationPoseRight(idStation);
    
    addCommands(
      // drive to first target, score, and set intake mode
      new DriveToTargetPose(drivetrain, nearScoringPoseLeft).withTimeout(4),
      elevator.score(ReefLevel.LEVEL_4),
      elevator.setTargetCommand(ReefLevel.INTAKE),

      // drive to station and wait for coral
      new DriveToTargetPose(drivetrain, stationPose).withTimeout(3),
      dispenser.waitForCoralCommand().withTimeout(2),
      elevator.setTargetCommand(ReefLevel.BASE),

      // drive to station-side reef side and score left
      new DriveToTargetPose(drivetrain, nearStationScoringPoseRight).withTimeout(4),
      elevator.score(ReefLevel.LEVEL_4),
      elevator.setTargetCommand(ReefLevel.INTAKE),
      
      // drive to station and wait for coral
      new DriveToTargetPose(drivetrain, stationPose).withTimeout(4),
      dispenser.waitForCoralCommand().withTimeout(2),
      elevator.setTargetCommand(ReefLevel.BASE),

      // drive to station-side reef side and score right
      new DriveToTargetPose(drivetrain, nearStationScoringPoseLeft).withTimeout(4),
      elevator.score(ReefLevel.LEVEL_4)
    );
  }
}
