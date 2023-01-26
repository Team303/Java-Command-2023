package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveToPositionCommand extends CommandBase {
    
    private final double shoulderAngle;
    private final double elbowAngle;
    private final double clawAngle;

    public MoveToPositionCommand(double shoulderAngle, double elbowAngle, double clawAngle) {
        addRequirements(arm);
        this.shoulderAngle = shoulderAngle;
        this.elbowAngle = elbowAngle;
        this.clawAngle = clawAngle;
    }

    @Override
    public void execute() {
        arm.reach(
                new double[] { shoulderAngle, elbowAngle, clawAngle });
    }

}
