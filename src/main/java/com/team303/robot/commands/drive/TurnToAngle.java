package com.team303.robot.commands.drive;

import com.team303.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class TurnToAngle extends PIDCommand {

    public TurnToAngle(double angle) {
        super(new PIDController(0.07, 0, 0),
                () -> Robot.getNavX().getAngle() % 360.0,
                angle,
                (output) -> Robot.swerve.drive(new Translation2d(0, 0), output, true),
                Robot.swerve);
        getController().setTolerance(5);
        getController().enableContinuousInput(-180, 180);
    }

    @Override
    public boolean isFinished() {
        System.out.println("Finished!!!\n\n\n\n");
        return getController().atSetpoint();
    }
}