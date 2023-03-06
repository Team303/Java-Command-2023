package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendToIntakeCommand extends CommandBase {
    List<Double> desiredAngles = Arrays.asList(0.0,90.0,90.0);

    public ExtendToIntakeCommand() {
        addRequirements(arm);
    }

    @Override
    public void execute() {
        // TODO: Find optimal angles
        arm.reach(desiredAngles);
    }
}