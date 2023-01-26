// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team303.robot.commands.led;

import static com.team303.robot.Robot.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LEDLeftRight extends CommandBase {
	private Color left;
	private Color right;

	public LEDLeftRight(Color left, Color right) {
		addRequirements(leds);

		this.left = left;
		this.right = right;
	}

	@Override
	public void initialize() {
		leds.clear();
	}

	@Override
	public void execute() {
		int bufferLen = leds.ledBuffer.getLength() / 2;

		for (int i = 0; i < bufferLen; i++) {
			leds.ledBuffer.setLED(i, this.left);
			leds.ledBuffer.setLED(bufferLen + i, this.right);
		}

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
