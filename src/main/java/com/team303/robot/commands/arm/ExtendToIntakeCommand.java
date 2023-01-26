package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendToIntakeCommand extends CommandBase {

    public ExtendToIntakeCommand() {
        addRequirements(arm);
    }

    @Override
    public void execute() {
        // TODO: Find optimal angles
        arm.reach(
                new double[] { Math.toRadians(0), Math.toRadians(90), Math.toRadians(90) });
    }
}
