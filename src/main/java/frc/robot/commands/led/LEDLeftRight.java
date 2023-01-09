// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.LEDSubsystem;

public class LEDLeftRight extends CommandBase {
	private Color left;
	private Color right;

	public LEDLeftRight(Color left, Color right) {
		addRequirements(LEDSubsystem.getLED());

		this.left = left;
		this.right = right;
	}

	@Override
	public void initialize() {
		LEDSubsystem.getLED().clear();
	}

	@Override
	public void execute() {
		int bufferLen = LEDSubsystem.getLED().ledBuffer.getLength() / 2;

		for (int i = 0; i < bufferLen; i++) {
			LEDSubsystem.getLED().ledBuffer.setLED(i, this.left);
			LEDSubsystem.getLED().ledBuffer.setLED(bufferLen + i, this.right);
		}

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
