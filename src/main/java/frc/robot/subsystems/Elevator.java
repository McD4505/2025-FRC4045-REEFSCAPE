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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AddressableLedStrip.LEDState;

public class Elevator extends SubsystemBase {

  public enum ReefLevel {
    BASE, INTAKE, LEVEL_3, LEVEL_4
  }
  /** Creates a new Elevator. */
  private SparkMax lift = new SparkMax(15, MotorType.kBrushless);
  private SparkMax liftFollower = new SparkMax(16, MotorType.kBrushless);

  private Dispenser dispenser = new Dispenser();

  private SparkClosedLoopController elevatorController = lift.getClosedLoopController();

  private final double gearRatio = 10.71/1;  // rot_motor/rot_pulley
  private final double sprocketRadius = Units.inchesToMeters(0.875);  // meters

  private final double sprocketCircumfrence = 2 * Math.PI * sprocketRadius;  // meters/rot_pulley

  private final double elevatorToChain = 3/1;
  private final double conversionFactor = elevatorToChain * sprocketCircumfrence / gearRatio;  // meters/rot_motor

  private final double heightOffset = -0.05;

  private final double baseHeight = 0.03;
  private final double level3Height = 1.18;
  private final double level4Height = 1.95;

  private final double scoringOffset = 0.0;

  private final double baseSetpoint = baseHeight;
  private final double intakeSetpoint = 0.1 + heightOffset;

  private final double level3Setpoint = level3Height + heightOffset + scoringOffset;
  private final double level4Setpoint = level4Height + heightOffset + scoringOffset;

  private ReefLevel level = ReefLevel.BASE;

  private AddressableLedStrip leds = new AddressableLedStrip(0, 30);

  public Elevator() {
    configureLift();
    configureLiftFollower();
  }

  private void configureLift() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
      .inverted(false)
      .idleMode(IdleMode.kBrake);

    config.encoder
      .positionConversionFactor(conversionFactor)
      .velocityConversionFactor(conversionFactor);

    config.closedLoop
      .p(2)
      .i(0)
      .d(0)
      .outputRange(-0.2, 0.5);

    lift.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command setDispenserSpeed(double mps) {
    return runOnce(() -> {
      dispenser.setSpeed(mps);
    });
  }

  private void configureLiftFollower() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.follow(lift, false);

    liftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    if(level == ReefLevel.BASE && Math.abs(lift.getEncoder().getPosition() - baseSetpoint) < 0.3) {
      disablePID();
    }

    if(dispenser.hasCoral()) {
      leds.setState(LEDState.GREEN);
    } else {
      leds.setState(LEDState.RED);
    }

    if(dispenser.isLimitSwitchPressed() && isElevatorAtBase() && level == ReefLevel.BASE) {
      zeroAngleMotor();
    }
  }

  public boolean isElevatorAtBase() {
    return Math.abs(getHeight()) < 0.03;
  }

  public double getHeight() {
    return lift.getEncoder().getPosition();
  }

  public void setHeight(double height) {
    elevatorController.setReference(height, ControlType.kPosition);
  }

  public void disablePID() {
    elevatorController.setReference(0, ControlType.kDutyCycle);
  }

  public void resetPosition() {
    lift.getEncoder().setPosition(0);
  }

  public void setTarget(ReefLevel level) {
    this.level = level;

    switch (level) {
      case BASE:
        setHeight(baseSetpoint);
        break;
      case INTAKE:
        setHeight(intakeSetpoint);
        break;
      case LEVEL_3:
        setHeight(level3Setpoint);
        break;
      case LEVEL_4:
        setHeight(level4Setpoint);
        break;
    }
  }

  public Command setTargetCommand(ReefLevel level) {
    return runOnce(() -> {
      setTarget(level);
      dispenser.setAngleTarget(level);
    });
  }

  public Command zeroAngleMotor() {
    return runOnce(() -> {
      dispenser.zeroAngleMotor();
    });
  }

  public Dispenser getDispenser() {
    return dispenser;
  }
}
