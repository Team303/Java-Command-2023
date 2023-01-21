package com.team303.robot.commands.arm;

import com.team303.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractToDefaultCommand extends CommandBase {
    public RetractToDefaultCommand() {
        addRequirements(ArmSubsystem.getArm());
    }
    public void execute() {
        ArmSubsystem.getArm().reach(
            //TODO: Find optimal angles
            new double[]{Math.toRadians(-45),Math.toRadians(135),Math.toRadians(135)}
        );
    }
}
