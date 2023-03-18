package com.team303.robot.commands.claw;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;

public class DefaultClaw extends CommandBase {
    public DefaultClaw() {
        addRequirements(Robot.claw);
    }

    @Override
    public void execute() {

        // Robot.claw.setRotateSpeed(MathUtil.applyDeadband(Robot.getOperatorXbox().getRightX(), 0.05));
        Robot.claw.setClawSpeed(MathUtil.applyDeadband(Robot.getOperatorXbox().getLeftX(), 0.05) * 0.3);
    }
}
