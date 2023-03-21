package com.team303.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import com.team303.robot.Robot;
import com.team303.robot.RobotMap;

public class LEDFlash extends CommandBase {

	private Timer timer;
	private boolean isPrimary = true;
	private double period;

	public LEDFlash(double period) {
		addRequirements(ledStrip);

		timer = new Timer();
		this.period = period;
	}

	public void initialize() {
		timer.start();
	}

	public void execute() {
		// check the current time
		double time = timer.get();

		// if the time is less then the period do nothing / keep same color
		if (time <= period)
			return;

		// else reset the timer and turn it back on
		timer.reset();
		timer.start();

		// change the color
		isPrimary = !isPrimary;

		for (var i = 0; i < ledStrip.ledBuffer.getLength(); i++) {
			// a fancy way of saying if isPrimary is true use Flash_Primary color else use
			// Flash_Secondary color
			Color ledColor = isPrimary ? RobotMap.LED.FLASH_PRIMARY : RobotMap.LED.FLASH_SECONDARY;
			ledStrip.ledBuffer.setLED(i, ledColor);
		}

		// Send the data to LEDSubsytem
		ledStrip.writeData();
	}

	public void end(boolean interupted) {
		// stop the timer so it doesn't keep going
		timer.stop();
		timer.reset();
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
}