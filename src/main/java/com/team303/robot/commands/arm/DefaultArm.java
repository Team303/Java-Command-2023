package com.team303.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;

public class DefaultArm extends CommandBase {

    public static final double MAX_SPEED = 0.5;
    public static final double CONSTANT_VOLTAGE = -0.04;

    public DefaultArm() {
        addRequirements(Robot.arm);
    }

    @Override
    public void execute() {
        Robot.arm.move(
                (-Robot.operatorController.getRightTriggerAxis()
                + Robot.operatorController.getLeftTriggerAxis()) * MAX_SPEED,
                Robot.operatorController.getLeftY() * MAX_SPEED + CONSTANT_VOLTAGE,
                Robot.operatorController.getRightY() * MAX_SPEED);
    }
}
