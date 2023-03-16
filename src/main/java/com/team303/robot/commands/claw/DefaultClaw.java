package com.team303.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;

public class DefaultClaw extends CommandBase {
    public DefaultClaw() {
        addRequirements(Robot.claw);
    }

    @Override
    public void execute() {
        Robot.claw.setRotateSpeed(Robot.getOperatorCommandXbox().getRightX() );
        Robot.claw.setClawSpeed(Robot.getOperatorCommandXbox().getLeftX() * 0.5);
    }
}
