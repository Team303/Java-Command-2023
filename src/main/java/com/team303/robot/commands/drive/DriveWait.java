package com.team303.robot.commands.drive;

import com.team303.robot.Robot;
import com.team303.robot.commands.WaitCommand;
import com.team303.robot.subsystems.SwerveSubsystem;

public class DriveWait extends WaitCommand {

	public DriveWait(double seconds) {
		super(seconds, SwerveSubsystem.getSwerve());
	}

}