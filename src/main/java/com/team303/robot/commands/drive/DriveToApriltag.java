package com.team303.robot.commands.drive;

import com.team303.robot.Robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToApriltag extends CommandBase {

    private final PIDController FeedbackController = new PIDController(0.025, 0, 0.01);

    public DriveToApriltag() {
        FeedbackController.enableContinuousInput(-1, 1);
        FeedbackController.setTolerance(2, 0);
        addRequirements(Robot.swerve);
    }

    @Override
    public void execute() {
        Robot.swerve.drive(
                new Translation2d(0, FeedbackController.calculate(-Robot.photonvision.getDistanceToTarget(), 0)),
                0, true);
    }

    @Override
    public boolean isFinished() {
        return Robot.photonvision.getDistanceToTarget() <= 0.5; // Change the num
    }
}
