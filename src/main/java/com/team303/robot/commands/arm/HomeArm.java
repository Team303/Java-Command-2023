package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;;

public class HomeArm extends CommandBase {
    public HomeArm() {
        addRequirements(arm);
    }

    @Override
    public void execute() {
        if (!(arm.shoulderJoint.leftSwitchReverse.isPressed()
                || arm.shoulderJoint.rightSwitchReverse.isPressed())) {
            arm.shoulderJoint.setMotors(-0.1);
        }

        if (!arm.elbowJoint.switchReverse.isPressed()) {
            arm.elbowJoint.motor.set(0.1);
        }
    }

    @Override
    public boolean isFinished() {
        return (arm.shoulderJoint.leftSwitchReverse.isPressed()
                || arm.shoulderJoint.rightSwitchReverse.isPressed())
                && arm.elbowJoint.switchReverse.isPressed();
    }

    @Override
    public void end(boolean interrupted) {
        final double shoulderStartAngle = -20;
        final double elbowStartAngle = 170.0;
        final double wristStartAngle = 0.0;

        arm.setEncoders(shoulderStartAngle, elbowStartAngle, wristStartAngle);
    }
}
