package com.team303.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;

public class ToggleClose extends CommandBase {
    public ToggleClose() {
        addRequirements(Robot.claw);
    }

    @Override
    public void execute() {
        Robot.claw.setRotateSpeed(-0.5);
    }

    @Override
    public boolean isFinished() {
        return Robot.claw.innerLimitReached();
    }
}