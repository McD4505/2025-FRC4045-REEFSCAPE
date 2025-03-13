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
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AddressableLedStrip.LEDState;

public class Elevator extends SubsystemBase {

  public enum ReefLevel {
    STOWED, BASE, INTAKE, LEVEL_2, LEVEL_3, LEVEL_4
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
  private final double level4Height = 1.86;

  private final double clearIntakeHeight = Units.inchesToMeters(27);

  private final double scoringOffsetL23 = 0.12;  // L2 and L3 have the same angle
  private final double scoringOffsetL4 = 0.13;  // L4 has a different angle

  private final double baseSetpoint = baseHeight;
  private final double intakeSetpoint = baseHeight;

  private final double level2Setpoint = level2Height + scoringOffsetL23;
  private final double level3Setpoint = level3Height + scoringOffsetL23;
  private final double level4Setpoint = level4Height + scoringOffsetL4;

  private ReefLevel level = ReefLevel.BASE;

  private double heightSetpoint = baseSetpoint;

  private AddressableLedStrip leds = new AddressableLedStrip(0, 60);

  private boolean hasCoral = dispenser.hasCoral();

  public Elevator() {
    configureLift();
    configureLiftFollower();
  }

  private void configureLift() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30);

    config.encoder
      .positionConversionFactor(conversionFactor)
      .velocityConversionFactor(conversionFactor);

    config.closedLoop
      .p(1.5)
      .i(0.000001)
      .d(0)
      .outputRange(-0.2, 1);

    lift.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
    resetPosition();
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

    // update LEDs if state has changed
    boolean dispenserHasCoral = dispenser.hasCoral();
    if(dispenserHasCoral != hasCoral) {

      if(dispenserHasCoral) {
        leds.setState(LEDState.GREEN);
      } else {
        leds.setState(LEDState.RED);
      }

      hasCoral = dispenserHasCoral;
    }

    // zero angle motor if limit switch is pressed
    if(dispenser.isLimitSwitchPressed() && level == ReefLevel.BASE) {
      dispenser.zeroAngleMotor();
    }

    // update dispenser state (avoiding intake while going up)
    updateDispenserState();

    // update elevator state (waiting for dispenser before extending)
    // updateElevatorState();
  }

  public void setLEDIntakeMode() {
    if(dispenser.hasCoral()) {
      leds.setState(LEDState.GREEN);
    } else {
      leds.setState(LEDState.RED);
    }
  }

  public boolean isElevatorAtBase() {
    return getHeight() < 0.03;
  }

  public double getHeight() {
    return lift.getEncoder().getPosition();
  }

  public void setHeight(double height) {
    heightSetpoint = height;
    elevatorController.setReference(height, ControlType.kPosition);
  }

  public boolean elevatorAndDispenserAtSetpoint() {
    return atSetpoint() && getDispenser().atSetpoint();
  }

  /**
   * Command sequence that sets the elevator target and ends when it reaches its setpoint
   * @param level target level
   * @return the command to be scheduled
   */
  public Command waitToReachSetpointCommand() {
    // return Commands.waitUntil(() -> elevatorAndDispenserAtSetpoint());
    return Commands.waitUntil(() -> atSetpoint());
  }

  /**
   * Command sequence that sets elevator target, waits for it to reach setpoint, and dispenses coral
   * @param level target level
   * @return the command to be scheduled
   */
  public Command score(ReefLevel level) {
    return Commands.sequence(
      setTargetCommand(level),
      waitToReachSetpointCommand().withTimeout(3),
      dispenser.dispenseCommand(),
      setTargetCommand(ReefLevel.BASE),
      waitToReachSetpointCommand().withTimeout(3)
    );
  }

  private double getElevatorHeightPercentage() {
    return (getHeight() - baseHeight) / (level4Height - baseHeight);
  }

  public double getMaxSpeedFactor() {
    if(getHeight() < 0.1) return 1;
    return Math.max(0.1, 1 - getElevatorHeightPercentage());
  }

  public void disablePID() {
    elevatorController.setReference(0, ControlType.kDutyCycle);
  }

  public void resetPosition() {
    lift.getEncoder().setPosition(baseHeight);
  }

  public Command resetLiftCommand() {
    return runOnce(() -> {
      resetElevator();
    });
  }

  /**
   * Updates elevator height target based on level
   * @param level target level
   */
  public void setTarget(ReefLevel level) {
    this.level = level;

    switch (level) {
      case STOWED:
        setHeight(baseSetpoint);
        break;
      case BASE:
        leds.setState(LEDState.RAINBOW);
        setHeight(baseSetpoint);
        break;
      case INTAKE:
        setLEDIntakeMode();
        setHeight(intakeSetpoint);
        break;
      case LEVEL_2:
        leds.setState(LEDState.CYAN);
        setHeight(level2Setpoint);
        break;
      case LEVEL_3:
        leds.setState(LEDState.ORANGE);
        setHeight(level3Setpoint);
        break;
      case LEVEL_4:
        leds.setState(LEDState.PURPLE);
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
    return level != ReefLevel.INTAKE && !isClearOfIntake() && level != ReefLevel.STOWED;
  }

  private boolean shouldStowDispenser() {
    // if elevator is far from target level, stow dispenser
    // boolean stowL4 = level == ReefLevel.LEVEL_4 && getHeight() < level4Setpoint - 0.05;
    // boolean stowL3 = level == ReefLevel.LEVEL_3 && getHeight() < level3Setpoint - 0.05;

    // return isClearOfIntake() && (stowL4 || stowL3);
    return level == ReefLevel.STOWED && isElevatorAtBase();
  }

  /** 
   * Stow dispenser if necessary
   * otherwise, set dispenser angle to target level
  */
  public void updateDispenserState() {
    if(shouldAvoidIntake()) {
      dispenser.setAngleTarget(ReefLevel.BASE);

    // } else if(shouldStowDispenser()) {
    //   dispenser.setAngleTarget(ReefLevel.STOWED);

    } else {
      dispenser.setAngleTarget(level);
    }
    SmartDashboard.putBoolean("avoid intake", shouldAvoidIntake());
  }

  public void updateElevatorState() {
    if(shouldAvoidIntake() && !dispenser.atSetpoint()) {
      setTarget(ReefLevel.BASE);
    } else {
      setTarget(level);
    }
  }

  public boolean atSetpoint() {
    return Math.abs(getHeight() - heightSetpoint) < 0.02;
  }

  public void resetElevator() {
    if(level == ReefLevel.BASE && Math.abs(lift.getEncoder().getVelocity()) < 0.002) {
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
