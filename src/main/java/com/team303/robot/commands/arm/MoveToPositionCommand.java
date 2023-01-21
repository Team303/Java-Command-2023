package com.team303.robot.commands.arm;

import com.team303.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveToPositionCommand extends CommandBase {
    private final double shoulderAngle;
    private final double elbowAngle;
    private final double clawAngle;
        public MoveToPositionCommand(double shoulderAngle, double elbowAngle, double clawAngle) {
        addRequirements(ArmSubsystem.getArm());
        this.shoulderAngle = shoulderAngle;
        this.elbowAngle = elbowAngle;
        this.clawAngle = clawAngle;
    }
    public void execute() {
        ArmSubsystem.getArm().reach(
            new double[]{shoulderAngle,elbowAngle,clawAngle}
        );
    }

    
}
