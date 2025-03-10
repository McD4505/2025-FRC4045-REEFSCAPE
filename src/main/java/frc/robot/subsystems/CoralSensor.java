// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSensor extends SubsystemBase {
  /** Creates a new CoralSensor. */
  private static final int coralSensorPort = 8;
  private DigitalInput coralSensor;
  
  public CoralSensor() {
    this.coralSensor = new DigitalInput(coralSensorPort);
  }

  public boolean hasCoral() {
    return !coralSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("coral detected?", hasCoral());
  }
}
