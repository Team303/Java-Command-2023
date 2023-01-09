// Copyright (c) 2022 Team 303

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.LEDSubsystem;

public class LEDSolidColor extends CommandBase {
	Color color;

	public LEDSolidColor(Color color) {
		addRequirements(LEDSubsystem.getLED());
		this.color = color;
	}

	@Override
	public void initialize() {
		// for every LED set a color
		for (var i = 0; i < LEDSubsystem.getLED().ledBuffer.getLength(); i++) {
			LEDSubsystem.getLED().ledBuffer.setLED(i, this.color);
		}
		// Send color data to LEDSubsytem
		LEDSubsystem.getLED().writeData();
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
}
