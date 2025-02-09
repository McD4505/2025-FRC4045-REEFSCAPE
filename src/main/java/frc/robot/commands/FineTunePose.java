// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FineTunePose extends Command {
  private final CommandSwerveDrivetrain drivetrain;

  private final PIDController xController = new PIDController(0.1, 0, 0);
  private final PIDController yController = new PIDController(0.1, 0, 0);
  private final PIDController thetaController = new PIDController(0.1, 0, 0);

  private Pose2d targetPose = new Pose2d();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  /** Creates a new FineTunePose. */
  public FineTunePose(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPose = drivetrain.getTargetPose();
    xController.setSetpoint(targetPose.getX());
    yController.setSetpoint(targetPose.getY());
    thetaController.setSetpoint(targetPose.getRotation().getDegrees());

    xController.setTolerance(0.03);
    yController.setTolerance(0.03);
    thetaController.setTolerance(5);

    thetaController.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;

    double xSpeed = xController.calculate(currentPose.getX());
    double ySpeed = yController.calculate(currentPose.getY());
    double thetaSpeed = thetaController.calculate(currentPose.getRotation().getDegrees());
    
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
