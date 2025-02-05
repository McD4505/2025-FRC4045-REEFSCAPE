// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  private AddressableLED led;
  private AddressableLEDBuffer buffer;

  private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
  private static final Distance ledSpacing = Meters.of(1.0 / 120);

  private final LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.25), ledSpacing);

  public LEDs(int port) {
    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(60);
    led.setLength(buffer.getLength());
    // led.setData(buffer);
    led.start();

    setDefaultCommand(runPattern(scrollingRainbow));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    scrollingRainbow.applyTo(buffer);
    led.setData(buffer);
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(buffer));
  }
}
