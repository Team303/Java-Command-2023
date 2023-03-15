package com.team303.robot.commands.drive;

import static com.team303.robot.Robot.ALLIANCE_SUBSTATION_ID;
import static com.team303.robot.subsystems.SwerveSubsystem.MAX_DRIVE_SPEED;
import static com.team303.robot.RobotMap.IOConstants.DEADBAND_FILTER;
import com.team303.robot.Robot;
import com.team303.robot.RobotMap.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.subsystems.SwerveSubsystem;

public class DefaultDrive extends CommandBase {

    boolean fieldOriented;

    public DefaultDrive(boolean fieldOriented) {
        addRequirements(Robot.swerve);
        this.fieldOriented = fieldOriented;
    }

    @Override
    public void execute() {
        if (Robot.controllerChooser.getSelected().equals("Controller")) {
                if (Robot.isReal())
                        Robot.swerve.drive(
                                new Translation2d(
                                        DEADBAND_FILTER.applyDeadband(Robot.getDriverXbox().getLeftY(), DEADBAND_FILTER.getLowerBound())
                                                * Swerve.MAX_VELOCITY * MAX_DRIVE_SPEED,
                                        DEADBAND_FILTER.applyDeadband(Robot.getDriverXbox().getLeftX(), DEADBAND_FILTER.getLowerBound())
                                                * Swerve.MAX_VELOCITY * MAX_DRIVE_SPEED),
                                DEADBAND_FILTER.applyDeadband(-Robot.getDriverXbox().getRightX(), DEADBAND_FILTER.getLowerBound()) * 4,
                                fieldOriented);
                else 
                        Robot.swerve.drive(
                                new Translation2d(
                                        DEADBAND_FILTER.applyDeadband(-Robot.getDriverXbox().getLeftY(), DEADBAND_FILTER.getLowerBound())
                                                * Swerve.MAX_VELOCITY * MAX_DRIVE_SPEED,
                                        DEADBAND_FILTER.applyDeadband(-Robot.getDriverXbox().getLeftX(), DEADBAND_FILTER.getLowerBound())
                                                * Swerve.MAX_VELOCITY * MAX_DRIVE_SPEED),
                                DEADBAND_FILTER.applyDeadband(Robot.getDriverXbox().getRightX(), DEADBAND_FILTER.getLowerBound()) * 4,
                                fieldOriented);    
        } else {
                Robot.swerve.drive(
                        new Translation2d(
                                DEADBAND_FILTER.applyDeadband(Robot.getLeftJoyStick().getY(), DEADBAND_FILTER.getLowerBound())
                                        * Swerve.MAX_VELOCITY * MAX_DRIVE_SPEED,
                                DEADBAND_FILTER.applyDeadband(-Robot.getLeftJoyStick().getX(), DEADBAND_FILTER.getLowerBound())
                                        * Swerve.MAX_VELOCITY * MAX_DRIVE_SPEED),
                        DEADBAND_FILTER.applyDeadband(-Robot.getRightJoyStick().getX(), DEADBAND_FILTER.getLowerBound()) * 4,
                        fieldOriented);
                }
        }
}
