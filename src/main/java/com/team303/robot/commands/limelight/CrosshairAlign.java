package com.team303.robot.commands.limelight;

import static com.team303.robot.Robot.limelight;
import static com.team303.robot.Robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CrosshairAlign extends CommandBase {

    public static PIDController xControl;
    public static PIDController yControl;

    public CrosshairAlign() {
        addRequirements(swerve);
        xControl = new PIDController(0.01, 0, 0);
        yControl = new PIDController(0.01, 0, 0);
    }

    @Override
    public void execute() {
        swerve.drive(
                new Translation2d(
                        xControl.calculate(limelight.getHorizontalCrosshairOffestAngle(), 0),
                        yControl.calculate(limelight.getVerticalCrosshairOffsetAngle(), 0)),
                0,
                true);
    }

    @Override
    public boolean isFinished() {
        return !limelight.hasValidTargets() || xControl.atSetpoint() && yControl.atSetpoint();
    }
}
