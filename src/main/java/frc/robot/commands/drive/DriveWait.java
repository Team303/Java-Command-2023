package frc.robot.commands.drive;

import frc.robot.commands.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveWait extends WaitCommand {

	public DriveWait(double seconds) {
		super(seconds, SwerveSubsystem.getSwerve());
	}

}