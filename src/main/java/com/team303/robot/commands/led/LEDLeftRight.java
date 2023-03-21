// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team303.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;

public class LEDLeftRight extends CommandBase {
	private Color left;
	private Color right;

	public LEDLeftRight(Color left, Color right) {
		addRequirements(ledStrip);

		this.left = left;
		this.right = right;
	}

	@Override
	public void initialize() {
		ledStrip.clear();
	}

	@Override
	public void execute() {
		int bufferLen = ledStrip.ledBuffer.getLength() / 2;

		for (int i = 0; i < bufferLen; i++) {
			ledStrip.ledBuffer.setLED(i, this.left);
			ledStrip.ledBuffer.setLED(bufferLen + i, this.right);
		}

		ledStrip.writeData();
	}

	@Override
	public void end(boolean interrupted) {
		ledStrip.clear();
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