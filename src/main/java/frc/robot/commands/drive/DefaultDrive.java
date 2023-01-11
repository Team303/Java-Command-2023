package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;

public class DefaultDrive extends CommandBase {

    boolean fieldOriented;

    public DefaultDrive(boolean fieldOriented) {
        addRequirements(SwerveSubsystem.getSwerve());
        this.fieldOriented = fieldOriented;
    }

    @Override
    public void execute() {
        SwerveSubsystem.getSwerve().drive(
            new Translation2d(
                Robot.getRightJoyStick().getX(), 
                Robot.getRightJoyStick().getY()
            ), 
            Robot.getLeftJoyStick().getY(),
            fieldOriented
        );
    }
}
