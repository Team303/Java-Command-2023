package com.team303.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;

public class RotateClaw extends CommandBase {

    private final double angle;
    private final double speed;


    public RotateClaw(double angle, double speed) {
        this.angle = angle;
        this.speed = speed;
        addRequirements(Robot.claw);
    }
    
    @Override
    public void execute() {
        Robot.claw.rotate(angle, speed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Robot.claw.getRotatePosition() - angle) < 1.0;
    }
}
