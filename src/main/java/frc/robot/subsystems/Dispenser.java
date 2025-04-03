// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ReefLevel;

public class Dispenser extends SubsystemBase {
  private SparkMax dispenser = new SparkMax(4, MotorType.kBrushless);
  private SparkMax angleMotor = new SparkMax(3, MotorType.kBrushless);

  private CoralSensor coralSensor = new CoralSensor();

  private SparkClosedLoopController controller = dispenser.getClosedLoopController();
  private SparkClosedLoopController angleController = angleMotor.getClosedLoopController();

  private final double wheelRadius = Units.inchesToMeters(2);
  private final double wheelGearRatio = 4/1;
  private final double conversionFactor = 2 * Math.PI * wheelRadius / wheelGearRatio / 60;  // meters/rot_motor

  private final double angleGearRatio = 15/1;

  private final double angleConversionFactor = 360 / angleGearRatio * 1.25;  // degrees/rot_motor

  private final double angleOffset = 300;

  private final double angleSetpointStowed = 0 + angleOffset;
  private final double angleSetpointBase = 66 + angleOffset;
  private final double angleSetpointIntake = 35 + angleOffset;
  private final double angleSetpointLevel2and3 = 28 + angleOffset;
  private final double angleSetpointLevel4 = 45 + angleOffset;

  private DigitalInput limitSwitch = new DigitalInput(9);

  // private AnalogInput distanceSensor = new AnalogInput(0);

  private ReefLevel level = ReefLevel.BASE;

  private double angleSetpoint = angleSetpointBase;

  /** Creates a new Dispenser. */
  public Dispenser() {
    configureDispenser();
    configureAngleMotor();

    zeroAngleMotor();
    setAngleTarget(ReefLevel.BASE);
  }

  public void zeroAngleMotor() {
    angleMotor.getEncoder().setPosition(0);
  }

  private void configureDispenser() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
      .inverted(true)
      .idleMode(IdleMode.kBrake);
      
    config.encoder
      .positionConversionFactor(conversionFactor)
      .velocityConversionFactor(conversionFactor);

    config.closedLoop
    .p(0.1)
    .i(0.001)
    .d(0)
    .iMaxAccum(0.5);

    dispenser.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void configureAngleMotor() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
      .inverted(false)
      .idleMode(IdleMode.kBrake);

    config.encoder
      .positionConversionFactor(angleConversionFactor)
      .velocityConversionFactor(angleConversionFactor);

    config.closedLoop
      .p(0.01)
      .i(0.0001)
      .d(0)
      .maxOutput(0.25)
      .minOutput(-0.25)
      .iZone(2)
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(angleOffset - 360, angleOffset);

    config.absoluteEncoder
      .setSparkMaxDataPortConfig()
      .positionConversionFactor(360)
      .velocityConversionFactor(360);

    angleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSpeed(double percent) {
    controller.setReference(percent, ControlType.kVelocity);
  }

  public void setAngle(double angle) {
    angleSetpoint = angle;
    angleController.setReference(angle, ControlType.kPosition);
  }

  public RelativeEncoder getAngleEncoder() {
    return angleMotor.getEncoder();
  }

  /**
   * sets the angle target based on level
   * @param level target level
   */
  public void setAngleTarget(ReefLevel level) {
    this.level = level;

    switch (level) {
      case DISABLED:
        setAngle(angleSetpointBase);
      case STOWED:
        setAngle(angleSetpointStowed);
        break;
      case BASE:
        setAngle(angleSetpointBase);
        break;
      case INTAKE:
        setAngle(angleSetpointIntake);
        break;
      case LEVEL_2:
        setAngle(angleSetpointLevel2and3);
        break;
      case LEVEL_3:
        setAngle(angleSetpointLevel2and3);
        break;
      case LEVEL_4:
        setAngle(angleSetpointLevel4);
        break;
      case HIGH:
        setAngle(angleSetpointLevel4);
        break;
    }
  }

  public Command setAngleTargetCommand(ReefLevel level) {
    return runOnce(() -> setAngleTarget(level));
  }

  public Command setSpeedCommand(double percent) {
    return runOnce(() -> setSpeed(percent));
  }

  public void stop() {
    controller.setReference(0, ControlType.kDutyCycle);
  }

  public boolean hasCoral() {
    return coralSensor.hasCoral();
  }

  public boolean isLimitSwitchPressed() {
    return !limitSwitch.get();
  }

  public double getAngle() {
    return angleMotor.getAbsoluteEncoder().getPosition();
  }

  public boolean atSetpoint() {
    return Math.abs(getAngle() - angleSetpoint) < 2;
  }

  /**
   * Command that waits until coral is detected
   * @return the command to be scheduled
   */
  public Command waitForCoralCommand() {
    return Commands.waitUntil(this::hasCoral);
  }

  /**
   * Command that dispenses coral
   * @return the command to be scheduled
   */
  public Command dispenseCommand() {
      return Commands.sequence(
        setSpeedCommand(4),
        Commands.waitUntil(() -> !hasCoral()).withTimeout(2),
        Commands.waitSeconds(0.5),
        setSpeedCommand(0)
      );
  }

  public Command zeroAngleMotorCommand() {
    return new InstantCommand(() -> zeroAngleMotor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("angle encoder", getAngleEncoder().getPosition());
    SmartDashboard.putNumber("angle velocity", getAngleEncoder().getVelocity());
    SmartDashboard.putBoolean("limit switch", isLimitSwitchPressed());
    SmartDashboard.putNumber("absolute encoder", getAngle());
    SmartDashboard.putNumber("dispenser velocity", dispenser.getEncoder().getVelocity());
  }
}
