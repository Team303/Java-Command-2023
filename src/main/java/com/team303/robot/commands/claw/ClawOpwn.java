package com.team303.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;

public class ClawOpwn extends CommandBase {
    public ClawOpwn() {
        addRequirements(Robot.claw);
    }

    @Override
    public void execute() {
        Robot.claw.setClawSpeed(-0.2);
    }

    @Override
    public boolean isFinished() {
        return Robot.claw.outerLimitReached();
    }
}
