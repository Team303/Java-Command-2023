// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team303.robot.commands.led;

import static com.team303.robot.Robot.leds;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LEDRainbowRotate extends CommandBase {
	private int hue = 0;

	public LEDRainbowRotate() {
		addRequirements(leds);
	}

	@Override
	public void initialize() {
		leds.clear();
	}

	@Override
	public void execute() {

		int hue = this.hue++;

		// for each singlar LED a assign a color
		for (var i = 0; i < leds.ledBuffer.getLength(); i++) {
			leds.ledBuffer.setHSV(i, (hue + i) % 180, 255, 10);
		}

		// send the color to be used by the LEDSubsystem
		leds.writeData();
	}

	@Override
	public void end(boolean interrupted) {
		leds.clear();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
}
