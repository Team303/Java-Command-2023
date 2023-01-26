package com.team303.robot.commands.drive;

import static com.team303.robot.Robot.swerve;

import com.team303.robot.Robot;
import com.team303.robot.RobotMap;
import com.team303.robot.RobotMap.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class Autolevel extends PIDCommand {

    private static final PIDController GYRO_PID_CONTROLLER = new PIDController(0.01, 0, 0);

    public Autolevel(double gyro) {
        super(
                GYRO_PID_CONTROLLER,
                () -> -Robot.getNavX().getRawGyroY(),
                gyro,
                (output) -> swerve.drive(new Translation2d(0, output), 0.0, true),
                swerve);

        getController().setTolerance(2, 0);
        getController().enableContinuousInput(-Swerve.MAX_VELOCITY, Swerve.MAX_VELOCITY);
    }
} //ARYA IS LAME 4 REALSIES

