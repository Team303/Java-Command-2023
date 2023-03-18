package com.team303.robot.commands.claw;

import static com.team303.robot.Robot.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateClaw extends CommandBase {

    private final double angle;
    private final double speed;

    public RotateClaw(double angle, double speed) {
        this.angle = angle;
        this.speed = speed;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.rotate(angle, speed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(claw.getRotatePosition() - angle) < 1.0;
    }
}
