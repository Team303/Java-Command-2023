package com.team303.robot.commands.drive;

import com.team303.robot.Robot;

import com.team303.robot.Robot;
import com.team303.robot.RobotMap;
import com.team303.robot.RobotMap.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class AutolevelPID extends PIDCommand {

    public AutolevelPID(double gyro) {
        super(
                new PIDController(0.1, 0, 0),
                () -> -Robot.getNavX().getPitch(),
                gyro,
                (output) -> Robot.swerve.drive(new Translation2d(0, output), 0.0, true),
                Robot.swerve);

        getController().setTolerance(2, 0);
        getController().enableContinuousInput(-180, 180);
    }
}

