package com.team303.robot.commands.drive;

import static com.team303.robot.Robot.swerve;

import com.team303.robot.Robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class Autolevel extends PIDCommand {

    private static final PIDController PID_CONTROLLER = new PIDController(0.01, 0, 0);

    public Autolevel(double gyro) {
        super(
                PID_CONTROLLER,
                () -> -Robot.getNavX().getRawGyroY(),
                gyro,
                (output) -> swerve.drive(new Translation2d(0, output), 0.0, true),
                swerve);

        getController().setTolerance(2, 0);
    }

    @Override
    public boolean isFinished() {
        // FIXME: Make sure angular velocity is near 0
        return getController().atSetpoint();
    }

}
