// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ReefLevel;

public class Dispenser extends SubsystemBase {
  private SparkMax dispenser = new SparkMax(3, MotorType.kBrushless);
  private SparkMax angleMotor = new SparkMax(4, MotorType.kBrushless);

  private CoralSensor coralSensor = new CoralSensor();

  private SparkClosedLoopController controller = dispenser.getClosedLoopController();
  private SparkClosedLoopController angleController = angleMotor.getClosedLoopController();

  private final double wheelRadius = Units.inchesToMeters(2);
  private final double conversionFactor = 2 * Math.PI * wheelRadius;

  private final double angleConversionFactor = 360;  // degrees/rot_motor

  private final double baseAngleSetpoint = 0;
  private final double intakeAngleSetpoint = 30;
  private final double level3AngleSetpoint = 30;
  private final double level4AngleSetpoint = 60;

  /** Creates a new Dispenser. */
  public Dispenser() {
    configureDispenser();
    configureAngleMotor();
  }

  private void configureDispenser() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
      .inverted(false)
      .idleMode(IdleMode.kBrake);

    config.encoder
      .positionConversionFactor(conversionFactor)
      .velocityConversionFactor(conversionFactor);

    dispenser.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configureAngleMotor() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
      .inverted(true)
      .idleMode(IdleMode.kBrake);

    config.encoder
      .positionConversionFactor(angleConversionFactor)
      .velocityConversionFactor(angleConversionFactor);

    angleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSpeed(double metersPerSecond) {
    controller.setReference(metersPerSecond, ControlType.kVelocity);
  }

  public void setAngle(double angle) {
    angleController.setReference(angle, ControlType.kPosition);
  }

  public void setAngleTarget(ReefLevel level) {
    switch (level) {
      case BASE:
        setAngle(baseAngleSetpoint);
        break;
      case INTAKE:
        setAngle(intakeAngleSetpoint);
        break;
      case LEVEL_3:
        setAngle(level3AngleSetpoint);
        break;
      case LEVEL_4:
        setAngle(level4AngleSetpoint);
        break;
    }
  }

  public Command setAngleTargetCommand(ReefLevel level) {
    return runOnce(() -> setAngleTarget(level));
  }

  public Command setSpeedCommand(double metersPerSecond) {
    return runOnce(() -> setSpeed(metersPerSecond));
  }

  public void stop() {
    controller.setReference(0, ControlType.kDutyCycle);
  }

  public boolean hasCoral() {
    return coralSensor.hasCoral();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
