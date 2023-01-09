// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.LEDSubsystem;

public class LEDRainbowRotate extends CommandBase {
	private int hue = 0;

	public LEDRainbowRotate() {
		addRequirements(LEDSubsystem.getLED());
	}

	@Override
	public void initialize() {
		LEDSubsystem.getLED().clear();
	}

	@Override
	public void execute() {

		int hue = this.hue++;

		// for each singlar LED a assign a color
		for (var i = 0; i < LEDSubsystem.getLED().ledBuffer.getLength(); i++) {
			LEDSubsystem.getLED().ledBuffer.setHSV(i, (hue + i) % 180, 255, 10);
		}

		// send the color to be used by the LEDSubsystem
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
