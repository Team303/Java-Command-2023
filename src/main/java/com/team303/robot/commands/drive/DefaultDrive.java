package com.team303.robot.commands.drive;

import com.team303.robot.Robot;
import com.team303.robot.subsystems.SwerveSubsystem;
import com.team303.robot.RobotMap.Swerve;
import static com.team303.robot.RobotMap.IOConstants.DEADBAND_FILTER;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
                DEADBAND_FILTER.applyDeadband(Robot.getRightJoyStick().getX(),DEADBAND_FILTER.getLowerBound()) * Swerve.MAX_VELOCITY, 
                DEADBAND_FILTER.applyDeadband(Robot.getRightJoyStick().getY(),DEADBAND_FILTER.getLowerBound()) * Swerve.MAX_VELOCITY
            ), 
            DEADBAND_FILTER.applyDeadband(Robot.getLeftJoyStick().getY(),DEADBAND_FILTER.getLowerBound()),
            fieldOriented
        );
    }
}
