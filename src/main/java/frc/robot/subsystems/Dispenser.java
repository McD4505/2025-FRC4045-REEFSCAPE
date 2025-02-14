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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dispenser extends SubsystemBase {
  private SparkMax max = new SparkMax(9, MotorType.kBrushless);
  private CoralSensor coralSensor = new CoralSensor();

  private SparkClosedLoopController controller = max.getClosedLoopController();

  private final double wheelRadius = Units.inchesToMeters(2);
  private final double conversionFactor = 2 * Math.PI * wheelRadius;

  /** Creates a new Dispenser. */
  public Dispenser() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
      .inverted(false)
      .idleMode(IdleMode.kBrake);

    config.encoder
      .positionConversionFactor(conversionFactor)
      .velocityConversionFactor(conversionFactor);

    max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSpeed(double metersPerSecond) {
    controller.setReference(metersPerSecond, ControlType.kVelocity);
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
