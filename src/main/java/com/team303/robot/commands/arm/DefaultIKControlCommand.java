package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.RobotMap.IOConstants.DEADBAND_FILTER;

import com.team303.robot.Robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultIKControlCommand extends CommandBase {

    public DefaultIKControlCommand() {
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.reach(
                new Translation3d(
                        DEADBAND_FILTER.applyDeadband(Robot.getXbox().getLeftX(), DEADBAND_FILTER.getLowerBound()),
                        0.0,
                        DEADBAND_FILTER.applyDeadband(Robot.getXbox().getLeftY(), DEADBAND_FILTER.getLowerBound())));

    }

}
