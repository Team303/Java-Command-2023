package com.team303.robot.commands.drive;

import com.team303.robot.Robot;

import com.team303.robot.Robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class TurnToAngle extends PIDCommand {

    public TurnToAngle(double angle) {
        super(new PIDController(0.01, 0, 0),
                () -> Robot.getNavX().getAngle(),
                angle,
                (output) -> Robot.swerve.drive(new Translation2d(0, 0), output, true),
                Robot.swerve);

        getController().setTolerance(0, 0);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}