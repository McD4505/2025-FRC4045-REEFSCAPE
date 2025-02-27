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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AddressableLedStrip.LEDState;

public class Elevator extends SubsystemBase {

  public enum ReefLevel {
    BASE, INTAKE, LEVEL_2, LEVEL_3, LEVEL_4
  }
  /** Creates a new Elevator. */
  private SparkMax lift = new SparkMax(15, MotorType.kBrushless);
  private SparkMax liftFollower = new SparkMax(16, MotorType.kBrushless);

  private Dispenser dispenser = new Dispenser();

  private SparkClosedLoopController elevatorController = lift.getClosedLoopController();

  private final double gearRatio = 10.71/1;  // rot_motor/rot_pulley
  private final double sprocketRadius = Units.inchesToMeters(0.875);  // meters

  private final double sprocketCircumference = 2 * Math.PI * sprocketRadius;  // meters/rot_pulley

  private final double elevatorToChain = 3/1;
  private final double conversionFactor = elevatorToChain * sprocketCircumference / gearRatio;  // meters/rot_motor
  
  private final double baseHeight = Units.inchesToMeters(9.875);
  private final double level2Height = 0.81;
  private final double level3Height = 1.21;
  private final double level4Height = 1.83;

  private final double clearIntakeHeight = Units.inchesToMeters(27);

  private final double scoringOffsetL23 = 0.15;  // L2 and L3 have the same angle
  private final double scoringOffsetL4 = 0.28;  // L4 has a different angle

  private final double baseSetpoint = baseHeight;
  private final double intakeSetpoint = baseHeight;

  private final double level2Setpoint = level2Height + scoringOffsetL23;
  private final double level3Setpoint = level3Height + scoringOffsetL23;
  private final double level4Setpoint = level4Height + scoringOffsetL4;

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
      .i(0.000000000004)
      .d(0)
      .outputRange(-0.15, 0.4);

    lift.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    lift.getEncoder().setPosition(baseHeight);
  }

  public Command setDispenserSpeedCommand(double mps) {
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
    // soften elevator landing
    if(level == ReefLevel.BASE && Math.abs(getHeight() - baseSetpoint) < 0.2) {
      disablePID();
    }

    // update LEDs
    if(dispenser.hasCoral()) {
      leds.setState(LEDState.GREEN);
    } else {
      leds.setState(LEDState.RED);
    }

    // zero angle motor if limit switch is pressed
    if(dispenser.isLimitSwitchPressed() && level == ReefLevel.BASE) {
      dispenser.zeroAngleMotor();
    }

    // update dispenser state (accounting for stowing)
    updateDispenserState();
  }

  public boolean isElevatorAtBase() {
    return getHeight() < 0.03;
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
    lift.getEncoder().setPosition(baseHeight);
  }

  /**
   * Updates elevator height target based on level
   * @param level target level
   */
  public void setTarget(ReefLevel level) {
    this.level = level;

    switch (level) {
      case BASE:
        setHeight(baseSetpoint);
        break;
      case INTAKE:
        setHeight(intakeSetpoint);
        break;
      case LEVEL_2:
        setHeight(level2Setpoint);
        break;
      case LEVEL_3:
        setHeight(level3Setpoint);
        break;
      case LEVEL_4:
        setHeight(level4Setpoint);
        break;
    }
  }

  public boolean isClearOfIntake() {
    return getHeight() > clearIntakeHeight;
  }

  /**
   * Use current height and target level to determine if dispenser should be stowed
   * @return true if dispenser should be stowed; false otherwise
   */
  private boolean shouldAvoidIntake() {
    return level != ReefLevel.INTAKE && !isClearOfIntake();
  }

  private boolean shouldStowDispenser() {
    // if elevator is far from target level, stow dispenser
    boolean stowL4 = level == ReefLevel.LEVEL_4 && getHeight() < level4Setpoint - 0.05;
    boolean stowL3 = level == ReefLevel.LEVEL_3 && getHeight() < level3Setpoint - 0.05;

    return isClearOfIntake() && (stowL4 || stowL3);
  }

  /** 
   * Stow dispenser if necessary
   * otherwise, set dispenser angle to target level
  */
  public void updateDispenserState() {
    if(shouldAvoidIntake()) {
      dispenser.setAngleTarget(ReefLevel.BASE);
    } else {
      dispenser.setAngleTarget(level);
    }
    SmartDashboard.putBoolean("avoid intake", shouldAvoidIntake());
  }

  public void resetElevator() {
    if(level == ReefLevel.BASE && Math.abs(lift.getEncoder().getVelocity()) < 0.001) {
      resetPosition();
    }
  }

  public Command setTargetCommand(ReefLevel level) {
    return runOnce(() -> {
      setTarget(level);
      dispenser.setAngleTarget(level);
    });
  }

  public Command zeroAngleMotorCommand() {
    return runOnce(() -> {
      dispenser.zeroAngleMotor();
    });
  }

  public Dispenser getDispenser() {
    return dispenser;
  }
}
