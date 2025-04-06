// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToTargetPose extends Command {
  private final CommandSwerveDrivetrain drivetrain;

  private final ProfiledPIDController xController = new ProfiledPIDController(
    3.5, 0, 0,
    new TrapezoidProfile.Constraints(4.5, 2.0) // max velocity 4.5 m/s, max acceleration 3.0 m/s^2
  );
  private final ProfiledPIDController yController = new ProfiledPIDController(
    3.5, 0, 0,
    new TrapezoidProfile.Constraints(4.5, 2.0)
  );
  private final ProfiledPIDController thetaController = new ProfiledPIDController(
    0.1, 0, 0,
    new TrapezoidProfile.Constraints(360, 90) // max velocity 360 deg/s, max acceleration 180 deg/s^2
  );

  private Pose2d targetPose = new Pose2d();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private boolean useDrivetrainTarget = true;

  private final double maxSpeed = 4.5; // m/s

  /** Creates a new DriveToTargetPose. */
  public DriveToTargetPose(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    useDrivetrainTarget = true;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // This constructor is more useful for auto and predefined positions
  public DriveToTargetPose(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    useDrivetrainTarget = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(useDrivetrainTarget) {
      targetPose = drivetrain.getTargetPose();
    }
    
    LimelightHelpers.setLEDMode_ForceOn("limelight-two");
    LimelightHelpers.setLEDMode_ForceOn("limelight");

    // reset controllers to clear stale data
    double normalizedAngle = drivetrain.getState().Pose.getRotation().getDegrees() % 360;
    while(normalizedAngle < 0) normalizedAngle += 360;
    xController.reset(drivetrain.getState().Pose.getX());
    yController.reset(drivetrain.getState().Pose.getY());
    thetaController.reset(normalizedAngle);

    // add setpoints
    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());
    thetaController.setGoal(targetPose.getRotation().getDegrees());

    // set tolerances (x, y in meters; angle in degrees)
    xController.setTolerance(0.02);
    yController.setTolerance(0.02);
    thetaController.setTolerance(0.2);

    // make angles 0 and 360 equivalent
    thetaController.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;

    // calculate speeds using profiled controllers
    double xSpeed = xController.calculate(currentPose.getX());
    double ySpeed = yController.calculate(currentPose.getY());

    // normalize pose angle to be between 0 and 360
    double normalizedAngle = currentPose.getRotation().getDegrees() % 360;
    while(normalizedAngle < 0) normalizedAngle += 360;
    
    double thetaSpeed = thetaController.calculate(normalizedAngle);
    
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
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }
}
