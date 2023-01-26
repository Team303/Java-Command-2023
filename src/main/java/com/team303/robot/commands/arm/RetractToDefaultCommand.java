package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractToDefaultCommand extends CommandBase {
    public RetractToDefaultCommand() {
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.reach(
                // TODO: Find optimal angles
                new double[] { Math.toRadians(-45), Math.toRadians(135), Math.toRadians(135) });
    }
}
