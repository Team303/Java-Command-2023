// Copyright (c) 2022 Team 303

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class WaitCommand extends CommandBase {
	private double waitTime;
	private Timer timer;

	public WaitCommand(double seconds, Subsystem... subsystems) {
		this.waitTime = seconds;

		addRequirements(subsystems);
	}

	@Override
	public void initialize() {
		timer.start();
	}

	@Override
	public void end(boolean interrupted) {
		timer.reset();
	}

	@Override
	public boolean isFinished() {
		// Command is done if timer has passed wait duration
		return timer.get() >= waitTime;
	}
}
