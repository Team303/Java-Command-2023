package com.team303.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;

public class DefaultMove extends CommandBase {
    public DefaultMove() {
        addRequirements(Robot.arm);
    }

    @Override
    public void execute() {
        Robot.arm.move(-Robot.getOperatorXbox().getRightTriggerAxis() + Robot.getOperatorXbox().getLeftTriggerAxis(),
            Robot.getOperatorXbox().getLeftY() - 0.10,
            Robot.getOperatorXbox().getRightY());
    }
}
