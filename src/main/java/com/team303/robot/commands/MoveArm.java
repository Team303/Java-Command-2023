package com.team303.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;

public class MoveArm extends CommandBase {
    public MoveArm() {
        addRequirements(Robot.arm);
    }

    @Override
    public void execute()
     {
        Robot.arm.move(Robot.getOperatorXbox().getRightTriggerAxis() -Robot.getOperatorXbox().getLeftTriggerAxis());
    }
}
