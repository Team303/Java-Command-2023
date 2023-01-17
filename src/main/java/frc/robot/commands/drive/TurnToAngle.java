package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class TurnToAngle extends PIDCommand {

    public TurnToAngle(double angle)
    {
        super(new PIDController(0.01, 0, 0),
            ()->Robot.getNavX().getAngle(),
            angle,
            (output) -> SwerveSubsystem.getSwerve().fieldoOrientedDrive(new Translation2d(0, 0), output),
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