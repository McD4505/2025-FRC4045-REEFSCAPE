// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  public enum ReefLevel {
    LEVEL_3, LEVEL_4
  }
  /** Creates a new Elevator. */
  private SparkMax lift1 = new SparkMax(8, MotorType.kBrushless);
  private SparkMax lift2 = new SparkMax(9, MotorType.kBrushless);

  private SparkClosedLoopController elevatorController = lift1.getClosedLoopController();

  private final double gearRatio = 1;  // rot_motor/rot_pulley
  private final double pulleyRadius = 0.02;  // meters

  private final double pulleyCircumfrence = 2 * Math.PI * pulleyRadius;  // = meters/rot_pulley

  private final double conversionFactor = pulleyCircumfrence / gearRatio;  // meters/rot_motor

  private final double heightOffset = 0.05;
  private final double level3Height = heightOffset + 1.27;
  private final double level4Height = heightOffset + 1.9;

  public Elevator() {
    SparkMaxConfig lift1Config = new SparkMaxConfig();

    lift1Config
      .inverted(false)
      .idleMode(IdleMode.kBrake);

    lift1Config.encoder
      .positionConversionFactor(conversionFactor)
      .velocityConversionFactor(conversionFactor);

    lift1Config.closedLoop
      .p(1)
      .i(0)
      .d(0);
    
    SparkMaxConfig lift2Config = new SparkMaxConfig();
    lift2Config.follow(lift1, true);

    lift2.configure(lift2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setHeight(double height) {
    elevatorController.setReference(height, ControlType.kPosition);
  }

  public void setTarget(ReefLevel level) {
    switch (level) {
      case LEVEL_3:
        setHeight(level3Height);
        break;
      case LEVEL_4:
        setHeight(level4Height);
        break;
    
      default:
        break;
    }
  }
}
