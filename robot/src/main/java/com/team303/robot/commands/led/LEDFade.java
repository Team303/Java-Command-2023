package com.team303.robot.commands.led;

import com.team303.robot.Robot;
import com.team303.robot.subsystems.LEDSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LEDFade extends CommandBase {

	private int blue = 0;
	private boolean backwards = false;

	public LEDFade() {
		addRequirements(LEDSubsystem.getLED());
	}

	@Override
	public void initialize() {
		blue = 0;
		backwards = false;

		// for each singlar LED a assign a initial color
		for (var i = 0; i < LEDSubsystem.getLED().ledBuffer.getLength(); i++) {
			LEDSubsystem.getLED().ledBuffer.setRGB(i, 0, 0, blue);
		}

		// send the color to be used by the LEDSubsystem
		LEDSubsystem.getLED().writeData();

	}

	@Override
	public void execute() {
		// checks what direction the colors are going
		if (!backwards)
			blue += 5;
		else
			blue -= 5;

		// then cheks what value the led is set to (0-255) and flips direction
		if (blue > 255) {
			blue = 255;
			backwards = true;
		} else if (blue < 0) {
			blue = 0;
			backwards = false;
		}

		// for each singlar LED a assign a color
		for (var i = 0; i < LEDSubsystem.getLED().ledBuffer.getLength(); i++) {
			LEDSubsystem.getLED().ledBuffer.setRGB(i, 0, 0, blue);
		}

		// send the color to be used by the LEDSubsystem
		LEDSubsystem.getLED().writeData();

	}

	@Override
	public void end(boolean interrupted) {

		// At the end do it all over agin but set it all back to black/off
		for (var i = 0; i < LEDSubsystem.getLED().ledBuffer.getLength(); i++) {
			LEDSubsystem.getLED().ledBuffer.setRGB(i, 0, 0, 0);
		}

		blue = 0;
		backwards = false;

		LEDSubsystem.getLED().writeData();
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
}
