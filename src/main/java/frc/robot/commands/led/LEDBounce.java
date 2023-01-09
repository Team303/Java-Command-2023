// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Robot;

public class LEDBounce extends CommandBase {

	private static final int LED_COUNT = 5;

	private int index = 0;
	private boolean backwards;
	private Color color;

	public LEDBounce(Color color) {
		addRequirements(LEDSubsystem.getLED());

		this.color = color;
	}

	@Override
	public void initialize() {
		LEDSubsystem.getLED().clear();
	}

	@Override
	public void execute() {
		int bufferLen = LEDSubsystem.getLED().ledBuffer.getLength() / 2;

		for (int i = 0; i < LED_COUNT; i++)
			LEDSubsystem.getLED().ledBuffer.setLED(index + i, new Color(0, 0, 0));

		for (int i = 0; i < LED_COUNT; i++)
			LEDSubsystem.getLED().ledBuffer.setLED(bufferLen * 2 - index - i - 1, new Color(0, 0, 0));

		if (backwards) {
			index--;

			if (index <= 0)
				backwards = false;
		} else {
			index++;

			if (index >= bufferLen - LED_COUNT)
				backwards = true;
		}

		for (int i = 0; i < LED_COUNT; i++)
			LEDSubsystem.getLED().ledBuffer.setLED(index + i, this.color);

		for (int i = 0; i < LED_COUNT; i++)
			LEDSubsystem.getLED().ledBuffer.setLED(bufferLen * 2 - index - i - 1, this.color);

		LEDSubsystem.getLED().writeData();
	}

	@Override
	public void end(boolean interrupted) {
		LEDSubsystem.getLED().clear();
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
