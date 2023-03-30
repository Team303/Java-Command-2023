package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.commands.arm.DefaultIKControlCommand.cartesianStorage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

public class HomeArm extends CommandBase {

    private static Timer timer;

    public HomeArm() {
        addRequirements(arm);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (!arm.shoulderJoint.switchReverse.isPressed()) {
            arm.shoulderJoint.setMotors(-0.2);
        }

        if (!arm.elbowJoint.switchForward.isPressed()) {
            arm.elbowJoint.motor.set(0.2);
        }

        if (!arm.wristJoint.switchReverse.isPressed()) {
            arm.wristJoint.motor.set(-0.3);
        }
    }

    @Override
    public boolean isFinished() {
        return (arm.shoulderJoint.switchReverse.isPressed()
                && arm.elbowJoint.switchForward.isPressed()
                // && arm.wristJoint.switchReverse.isPressed()
                ) 
                || timer.get() > 4;
    }

    @Override
    public void end(boolean interrupted) {
        final double shoulderStartAngle = -19.5;
        final double elbowStartAngle = 160.0;
        final double wristStartAngle = -100.0;

        cartesianStorage = new Translation3d(13, 0, 20);

        arm.setEncoders(shoulderStartAngle, elbowStartAngle, wristStartAngle);
        timer.stop();
    }
}
