package com.team303.robot.commands.drive;

import static com.team303.robot.Robot.ALLIANCE_SUBSTATION_ID;
import com.team303.robot.Robot;
import static com.team303.robot.RobotMap.IOConstants.DEADBAND_FILTER;

import com.team303.robot.Robot;
import com.team303.robot.RobotMap.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultDrive extends CommandBase {

    boolean fieldOriented;

    public DefaultDrive(boolean fieldOriented) {
        addRequirements(Robot.swerve);
        this.fieldOriented = fieldOriented;
    }

    @Override
    public void execute() {
        Robot.swerve.drive(
                new Translation2d(
                        DEADBAND_FILTER.applyDeadband(Robot.getXbox().getRightX()
                        , DEADBAND_FILTER.getLowerBound())
                                * Swerve.MAX_VELOCITY,
                        DEADBAND_FILTER.applyDeadband(Robot.getXbox().getRightY(), DEADBAND_FILTER.getLowerBound())
                                * Swerve.MAX_VELOCITY),
                DEADBAND_FILTER.applyDeadband(Robot.getXbox().getLeftX(), DEADBAND_FILTER.getLowerBound()),
                fieldOriented);
    }
}
