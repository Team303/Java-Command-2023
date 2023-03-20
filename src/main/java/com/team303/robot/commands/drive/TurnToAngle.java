package com.team303.robot.commands.drive;

import com.team303.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class TurnToAngle extends PIDCommand {

    double angle;

    public TurnToAngle(double angle) {
        super(new PIDController(0.07, 0, 0),
                () -> Robot.navX.getAngle() % 360.0,
                angle,
                (output) -> Robot.swerve.drive(new Translation2d(0, 0), output, true),
                Robot.swerve);
        getController().setTolerance(5);
        getController().enableContinuousInput(-180, 180);
        this.angle = angle;
    }

    @Override
    public boolean isFinished() {

        double useDegrees = Math.toDegrees(Robot.swerve.angle);
        useDegrees %= 360;
        if (useDegrees < 0) {
            useDegrees += 360;
        }
        System.out.println("angle: " + useDegrees);
        return Math.abs(this.angle - useDegrees) < 10.0 || getController().atSetpoint();
    }
}