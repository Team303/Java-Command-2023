package com.team303.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.RobotMap.Swerve;
import com.team303.robot.Robot;

public class AutolevelFeedforward extends CommandBase {
    private final PIDController FeedbackController = new PIDController(0.5,0,0);
    private final SimpleMotorFeedforward FeedforwardController = new SimpleMotorFeedforward(
        1, 3);
    
    public AutolevelFeedforward() {
        FeedbackController.enableContinuousInput(-Swerve.MAX_VELOCITY * Swerve.MAX_DRIVE_SPEED, 
            Swerve.MAX_VELOCITY * Swerve.MAX_DRIVE_SPEED);
        FeedbackController.setTolerance(2, 0);
        addRequirements(Robot.swerve);
    }

    @Override
    public void execute() {
        Robot.swerve.drive(new Translation2d(0,
            FeedbackController.calculate(-Robot.getNavX().getPitch(), 0)
            + FeedforwardController.calculate(-Robot.getNavX().getPitch())), 
            0, true);
    }
}
