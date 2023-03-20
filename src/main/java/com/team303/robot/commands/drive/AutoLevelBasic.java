package com.team303.robot.commands.drive;

import com.team303.robot.Robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoLevelBasic extends CommandBase {
    private final PIDController FeedbackController = new PIDController(0.025, 0, 0.01);
    private final SimpleMotorFeedforward FeedforwardController = new SimpleMotorFeedforward(
            1, 0.5);

    public AutoLevelBasic() {
        FeedbackController.enableContinuousInput(-1, 1);
        FeedbackController.setTolerance(2, 0);
        addRequirements(Robot.swerve);
    }

    @Override
    public void execute() {
        Robot.swerve.drive(new Translation2d(-(FeedbackController.calculate(-Robot.navX.getPitch(), 0)
        + FeedforwardController.calculate(-Robot.navX.getPitch())) * 0.04,0),
                0, true);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Robot.navX.getPitch()) < 4.80 || FeedbackController.atSetpoint();
    }
}
