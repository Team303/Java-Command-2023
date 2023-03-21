package com.team303.robot.commands.drive;

import static com.team303.robot.Robot.driverController;
import static com.team303.robot.Robot.swerve;
import static com.team303.robot.RobotMap.IOConstants.DEADBAND_FILTER;
import static com.team303.robot.subsystems.SwerveSubsystem.MAX_DRIVE_SPEED;

import com.team303.robot.Robot;
import com.team303.robot.RobotMap.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultDrive extends CommandBase {

        boolean fieldOriented;

        public DefaultDrive(boolean fieldOriented) {
                addRequirements(swerve);
                this.fieldOriented = fieldOriented;
        }

        @Override
        public void execute() {
                // Deadband and square inputs
                Translation2d translation = new Translation2d(
                                -Math.signum(driverController.getLeftY()) * Math.pow(DEADBAND_FILTER.applyDeadband(
                                                driverController.getLeftY(),
                                                DEADBAND_FILTER.getLowerBound()), 2)
                                                * Swerve.MAX_VELOCITY * MAX_DRIVE_SPEED,
                                -Math.signum(driverController.getLeftX()) * Math.pow(DEADBAND_FILTER.applyDeadband(
                                                driverController.getLeftX(),
                                                DEADBAND_FILTER.getLowerBound()), 2)
                                                * Swerve.MAX_VELOCITY
                                                * MAX_DRIVE_SPEED);

                double rotation = DEADBAND_FILTER.applyDeadband(
                                (Robot.isReal() ? -1 : 1) * Robot.driverController.getRightX(),
                                DEADBAND_FILTER.getLowerBound());

                swerve.drive(
                                translation,
                                rotation * 4 * MAX_DRIVE_SPEED,
                                fieldOriented);
        }
}
