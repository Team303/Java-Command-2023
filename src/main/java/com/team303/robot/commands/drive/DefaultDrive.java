package com.team303.robot.commands.drive;

import static com.team303.robot.RobotMap.IOConstants.DEADBAND_FILTER;
import static com.team303.robot.subsystems.SwerveSubsystem.MAX_DRIVE_SPEED;

import com.team303.robot.Robot;
import com.team303.robot.RobotMap.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultDrive extends CommandBase {

        boolean fieldOriented;

        public DefaultDrive(boolean fieldOriented) {
                addRequirements(Robot.swerve);
                this.fieldOriented = fieldOriented;
        }

        @Override
        public void execute() {
                Translation2d translation = new Translation2d(
                                DEADBAND_FILTER.applyDeadband(
                                                -Robot.driverController.getLeftY(),
                                                DEADBAND_FILTER.getLowerBound())
                                                * Swerve.MAX_VELOCITY * MAX_DRIVE_SPEED,
                                DEADBAND_FILTER.applyDeadband(
                                                -Robot.driverController.getLeftX(),
                                                DEADBAND_FILTER.getLowerBound())
                                                * Swerve.MAX_VELOCITY
                                                * MAX_DRIVE_SPEED);

                double rotation = DEADBAND_FILTER.applyDeadband(
                                (Robot.isReal() ? -1 : 1) * Robot.driverController.getRightX(),
                                DEADBAND_FILTER.getLowerBound());

                Robot.swerve.drive(
                                translation,
                                rotation * 4 * MAX_DRIVE_SPEED,
                                fieldOriented);
        }
}
