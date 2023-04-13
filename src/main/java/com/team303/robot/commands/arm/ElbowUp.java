package com.team303.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;

public class ElbowUp extends CommandBase {

    double angle;

    public ElbowUp(double angle) {
        this.angle = angle;
        addRequirements(Robot.arm);
    }

    @Override
    public void initialize() {
        Robot.arm.setEncodersDegrees(0, 0, 0);
    }

    @Override
    public void execute() {
        Robot.arm.elbowJoint.setSpeed(-0.2);
    }

    @Override
    public boolean isFinished() {
        return Robot.arm.elbowJoint.getEncoderPosition() < angle;
    }
}
