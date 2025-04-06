// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
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

    Pose2d scoringPose1 = Vision.getScoringPose(idTarget1, true);
    Pose2d scoringPose2 = Vision.getScoringPose(idTarget2, true);
    Pose2d scoringPose3 = Vision.getScoringPose(idTarget2, false);
    Pose2d stationPose = isOtherBarge ? Vision.getStationPoseLeft(idStation) : Vision.getStationPoseRight(idStation);

    double raiseElevatorDistance = 3;

    BooleanSupplier isCloseToReef1Supplier = () -> drivetrain.distanceTo(scoringPose1) < raiseElevatorDistance;
    BooleanSupplier isCloseToReef2Supplier = () -> drivetrain.distanceTo(scoringPose2) < raiseElevatorDistance;
    BooleanSupplier isCloseToReef3Supplier = () -> drivetrain.distanceTo(scoringPose3) < raiseElevatorDistance;

    addCommands(
      // drive to first target, score, and set intake mode
      new DriveToTargetPose(drivetrain, scoringPose1).withTimeout(4)
      .alongWith(Commands.waitUntil(isCloseToReef1Supplier).andThen(elevator.setTargetCommand(ReefLevel.LEVEL_4))),
      elevator.score(ReefLevel.LEVEL_4),
      elevator.setTargetCommand(ReefLevel.INTAKE),

      // drive to station and wait for coral
      new DriveToTargetPose(drivetrain, stationPose).withTimeout(3).raceWith(dispenser.waitForCoralCommand()),
      dispenser.waitForCoralCommand().withTimeout(2),
      elevator.setTargetCommand(ReefLevel.BASE),

      // drive to station-side reef side and score left
      new DriveToTargetPose(drivetrain, scoringPose3).withTimeout(4)
      .alongWith(Commands.waitUntil(isCloseToReef2Supplier).andThen(elevator.setTargetCommand(ReefLevel.LEVEL_4))),
      elevator.score(ReefLevel.LEVEL_4),
      elevator.setTargetCommand(ReefLevel.INTAKE),
      
      // drive to station and wait for coral
      new DriveToTargetPose(drivetrain, stationPose).withTimeout(4).raceWith(dispenser.waitForCoralCommand()),
      dispenser.waitForCoralCommand().withTimeout(2),
      elevator.setTargetCommand(ReefLevel.BASE),

      // drive to station-side reef side and score right
      new DriveToTargetPose(drivetrain, scoringPose2).withTimeout(4)
      .alongWith(Commands.waitUntil(isCloseToReef3Supplier).andThen(elevator.setTargetCommand(ReefLevel.LEVEL_4))),
      elevator.score(ReefLevel.LEVEL_4)
    );
  }
}
