// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToTargetPose extends Command {
  private final CommandSwerveDrivetrain drivetrain;

  private final PIDController xController = new PIDController(4, 0, 0);
  private final PIDController yController = new PIDController(4, 0, 0);
  private final PIDController thetaController = new PIDController(0.1, 0, 0);

  private Pose2d targetPose = new Pose2d();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  /** Creates a new DriveToTargetPose. */
  public DriveToTargetPose(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.targetPose = drivetrain.getTargetPose();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  public DriveToTargetPose(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setLEDMode_ForceOn("limelight-two");

    xController.setSetpoint(targetPose.getX());
    yController.setSetpoint(targetPose.getY());
    thetaController.setSetpoint(targetPose.getRotation().getDegrees());

    xController.setTolerance(0.01);
    yController.setTolerance(0.01);
    thetaController.setTolerance(0.2);

    thetaController.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;

    // calculate x and y speeds
    double xSpeed = xController.calculate(currentPose.getX());
    double ySpeed = yController.calculate(currentPose.getY());

    // normalize pose angle to be between 0 and 360
    double normalizedAngle = currentPose.getRotation().getDegrees() % 360;
    while(normalizedAngle < 0) normalizedAngle += 360;
    
    // calculate theta speed
    double thetaSpeed = thetaController.calculate(normalizedAngle);
    
    // drive with calculated speeds
    drivetrain.applyRequest(() ->
                drive.withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)
                    .withRotationalRate(thetaSpeed)
            ).execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> brake).execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}
