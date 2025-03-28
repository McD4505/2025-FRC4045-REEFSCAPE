// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AddressableLedStrip extends SubsystemBase {

	private final AddressableLED ledStrip;
	private final AddressableLEDBuffer ledBuffer;
	private final int length;

	public enum LEDState {
		RAINBOW,
		GREEN,
		BLUE,
		PURPLE,
		CYAN,
		RED,
		WHITE,
		ORANGE,
		FLAME,
		AMERICAN,
		ALLIANCE,
		OFF
	}

	private LEDState state;
	private int hue;

	public AddressableLedStrip(int port, int length) {

		this.length = length;

		ledStrip = new AddressableLED(port);
		ledBuffer = new AddressableLEDBuffer(length);

		ledStrip.setLength(ledBuffer.getLength());

		ledStrip.setData(ledBuffer);

		state = LEDState.RAINBOW;

		hue = 0;

		ledStrip.start();
	}

	@Override
	public void periodic() {

		if (DriverStation.isDisabled()) {
			// solid(new Color(255, 25, 0));
			rainbow();
		} else {

			switch (state) {
				case RAINBOW:
					rainbow();
					break;

				case GREEN:
					solid(Color.kGreen);
					break;

				case BLUE:
					solid(Color.kBlue);
					break;

				case PURPLE:
					solid(Color.kPurple);
					break;

				case CYAN:
					solid(Color.kCyan);
					break;

				case RED:
					solid(Color.kRed);
					break;
				case WHITE:
					solid(Color.kWhite);
					break;

				case ORANGE:
					solid(Color.kOrange);
					break;

				case FLAME:
					flame();
					break;

				case ALLIANCE:
					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						if (alliance.get() == Alliance.Red) {
							solid(Color.kRed);
						} else if (alliance.get() == Alliance.Blue) {
							solid(Color.kBlue);
						}
					} else {
						solid(Color.kOrange);
					}
					break;
				case AMERICAN:
					everyOther(Color.kBlue, Color.kRed);
					break;
				case OFF:
					solid(Color.kBlack);
			}
		}

		ledStrip.setData(ledBuffer);
	}

	public void setState(LEDState state) {
		this.state = state;
	}

	private void rainbow() {
		for (var i = 0; i < length; i++) {
			final var hue = (this.hue + (i * 180 / length)) % 180;
			ledBuffer.setHSV(i, hue, 255, 128);
		}
		this.hue += 3;
		this.hue %= 180;
	}

	private void solid(Color color) {
		for (var i = 0; i < length; i++) {
			ledBuffer.setLED(i, color);
		}
	}

	private void everyOther(Color color1, Color color2) {
		for (var i = 0; i < length; i++) {
			if (i % 2 == 0) {
				ledBuffer.setLED(i, color1);
			} else {
				ledBuffer.setLED(i, color2);
			}
		}
	}

	public void laser(Color color) {
		for (var i = 0; i < length; i++) {
			ledBuffer.setLED(i, color);
		}
		ledStrip.setData(ledBuffer);
	}

	private void flame() {
		for (var i = 0; i < length; i++) {
			// Generate a random brightness and hue for the flame effect
			int brightness = (int) (Math.random() * 128) + 128; // Random brightness between 128 and 255
			int hue = (int) (Math.random() * 180); // Random hue between 0 and 180
			ledBuffer.setHSV(i, hue, 255, brightness);
		}
	}
}
