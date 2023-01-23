package com.team303.robot.commands.arm;

import com.team303.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendToIntakeCommand extends CommandBase {
    public ExtendToIntakeCommand() {
        addRequirements(ArmSubsystem.getArm());
    }

    @Override
    public void execute() {
        ArmSubsystem.getArm().reach(
            //TODO: Find optimal angles
            new double[]{Math.toRadians(0),Math.toRadians(90),Math.toRadians(90)}
        );
    }
}

