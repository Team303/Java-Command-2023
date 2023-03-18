package com.team303.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;

public class DefaultMove extends CommandBase {

    public static final double MAX_SPEED = 0.5;
    public static final double CONSTANT_VOLTAGE = -0.07;

    public DefaultMove() {
        addRequirements(Robot.arm);
    }

    @Override
    public void execute() {
        Robot.arm.move(
                (-Robot.getOperatorXbox().getRightTriggerAxis()
                        + Robot.getOperatorXbox().getLeftTriggerAxis()) * MAX_SPEED,
                Robot.getOperatorXbox().getLeftY() * MAX_SPEED + CONSTANT_VOLTAGE,
                Robot.getOperatorXbox().getRightY() * MAX_SPEED);
    }
}
