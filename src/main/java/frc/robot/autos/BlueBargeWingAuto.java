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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToTargetPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Dispenser;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Elevator.ReefLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueBargeWingAuto extends SequentialCommandGroup {
  /** Creates a new BlueBargeWingAuto. */
  public BlueBargeWingAuto(CommandSwerveDrivetrain drivetrain, Elevator elevator, Dispenser dispenser, Supplier<Boolean> isRedSupplier) {
    // Add your commands in the addCommands() call, e.g.

    PathPlannerPath startToNearSide;
    PathPlannerPath nearSideToStation;
    PathPlannerPath stationToNearStationSide;
    PathPlannerPath nearStationSideToStation;
    try {
      startToNearSide = PathPlannerPath.fromPathFile("blue barge center to near angled side");
      nearSideToStation = PathPlannerPath.fromPathFile("near angled side to station");
      stationToNearStationSide = PathPlannerPath.fromPathFile("blue barge side station to near station angled side");
      nearStationSideToStation = PathPlannerPath.fromPathFile("near station angled side to blue barge side station");
    } catch (FileVersionException e) {
      e.printStackTrace();
      return;
    } catch (IOException e) {
      e.printStackTrace();
      return;
    } catch (ParseException e) {
      e.printStackTrace();
      return;
    }

    int idTarget1 = 20;
    int idTarget2 = 19;

    int idStation = 13;
    
    if(isRedSupplier.get()) {
      startToNearSide = startToNearSide.flipPath();
      nearSideToStation = nearSideToStation.flipPath();
      stationToNearStationSide = stationToNearStationSide.flipPath();
      nearStationSideToStation = nearStationSideToStation.flipPath();

      idTarget1 = 9;
      idTarget2 = 8;
      idStation = 2;
    }

    // Command targetNearLeft = new InstantCommand(() -> Vision.targetBranchFromId(drivetrain, idTarget1, true));
    // Command targetNearStationLeft = new InstantCommand(() -> Vision.targetBranchFromId(drivetrain, idTarget2, true));
    // Command targetNearStationRight = new InstantCommand(() -> Vision.targetBranchFromId(drivetrain, idTarget2, false));
    // Command targetStation = new InstantCommand(() -> Vision.targetStationFromId(drivetrain, idStation));

    Pose2d nearScoringPoseLeft = Vision.getScoringPose(idTarget1, true);
    Pose2d nearStationScoringPoseLeft = Vision.getScoringPose(idTarget2, true);
    Pose2d nearStationScoringPoseRight = Vision.getScoringPose(idTarget2, false);
    Pose2d stationPose = Vision.getStationPose(idStation);


    Command driveStartToNearSide = AutoBuilder.followPath(startToNearSide);
    Command driveNearSideToStation = AutoBuilder.followPath(nearSideToStation);
    Command driveStationToNearStationSide = AutoBuilder.followPath(stationToNearStationSide);
    Command driveNearStationSideToStation = AutoBuilder.followPath(nearStationSideToStation);
    
    addCommands(
      // drive to first target, score, and set intake mode
      driveStartToNearSide,
      new DriveToTargetPose(drivetrain, nearScoringPoseLeft),
      elevator.score(ReefLevel.LEVEL_4),
      elevator.setTargetCommand(ReefLevel.INTAKE),

      // drive to station and wait for coral
      driveNearSideToStation,
      new DriveToTargetPose(drivetrain, stationPose),
      dispenser.waitForCoralCommand(),

      // drive to station-side reef side and score left
      driveStationToNearStationSide,
      new DriveToTargetPose(drivetrain, nearStationScoringPoseLeft),
      elevator.score(ReefLevel.LEVEL_4),
      elevator.setTargetCommand(ReefLevel.INTAKE),
      
      // drive to station and wait for coral
      driveNearStationSideToStation,
      new DriveToTargetPose(drivetrain, stationPose),
      dispenser.waitForCoralCommand(),

      // drive to station-side reef side and score right
      driveStationToNearStationSide,
      new DriveToTargetPose(drivetrain, nearStationScoringPoseRight),
      elevator.score(ReefLevel.LEVEL_4)
    );
  }
}
