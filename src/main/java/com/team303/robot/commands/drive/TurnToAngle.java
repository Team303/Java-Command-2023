package com.team303.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import com.team303.robot.Robot;
import com.team303.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class TurnToAngle extends PIDCommand {

    public TurnToAngle(double angle)
    {
        super(new PIDController(0.01, 0, 0),
            ()->Robot.getNavX().getAngle(),
            angle,
            (output) -> SwerveSubsystem.getSwerve().drive(new Translation2d(0, 0), output,true),
            SwerveSubsystem.getSwerve()
        );
    

    getController().setTolerance(0, 0);
    }

    @Override
	public void initialize() {
		SwerveSubsystem.getSwerve().setEncoderDistance();
		//Robot.getNavX().reset();
	}

	@Override
	public boolean isFinished() {
		return getController().atSetpoint();
	}
}