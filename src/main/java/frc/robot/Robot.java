// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean useLimelight = true;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    // limelight pose estimation
    if(useLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation("limelight-two", headingDeg, 0, 0, 0, 0, 0);

      var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if (mt2 != null && mt2.tagCount > 0 && omegaRps < 2.0) {
        m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        m_robotContainer.drivetrain.addVisionMeasurement(mt2.pose, Utils.fpgaToCurrentTime(mt2.timestampSeconds));
      }

      var mt2_1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-two");
      if (mt2_1 != null && mt2_1.tagCount > 0 && omegaRps < 2.0) {
        m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        m_robotContainer.drivetrain.addVisionMeasurement(mt2_1.pose, Utils.fpgaToCurrentTime(mt2_1.timestampSeconds));
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
