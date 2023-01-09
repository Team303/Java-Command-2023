package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;

public class FieldOrientedDrive extends CommandBase {

    public FieldOrientedDrive() {
        addRequirements(SwerveSubsystem.getSwerve());
    }

    @Override
    public void execute() {
        SwerveSubsystem.getSwerve().fieldoOrientedDrive(
            new Translation2d(
                Robot.getRightJoyStick().getX(), 
                Robot.getRightJoyStick().getY()
            ), 
            Robot.getLeftJoyStick().getY()
        );
    }
}
