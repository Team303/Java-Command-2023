package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;;

public class Homing extends CommandBase {

    double shoulderStartAngle;
    double elbowStartAngle;
    double clawStartAngle;
    double speed;

    public Homing() {
        addRequirements(arm);
        System.out.println("\n\n\n\n\n\n\n\nik angles" + arm.armKinematics.getIKAnglesDegrees().get(0)
                + arm.armKinematics.getIKAnglesDegrees().get(1));
    }

    @Override
    public void execute() {
        if (!arm.shoulderJoint.shoulderSwitchReverse1.isPressed()) {
            arm.shoulderJoint.setMotors(-0.1);
        }

        if (!arm.elbowJoint.elbowSwitchReverse.isPressed()) {
            arm.elbowJoint.elbowMotor.set(0.1);
        }
    }

    @Override
    public boolean isFinished() {
        return arm.shoulderJoint.shoulderSwitchReverse1.isPressed() && arm.elbowJoint.elbowSwitchReverse.isPressed();
    }

    @Override
    public void end(boolean interrupted) {

        shoulderStartAngle = -20;
        elbowStartAngle = 170.0;

        arm.setEncoders(shoulderStartAngle, elbowStartAngle, 0);

        System.out.println("ik angles" + arm.armKinematics.getIKAnglesDegrees().get(0)
                + arm.armKinematics.getIKAnglesDegrees().get(1));

        // arm.reach(arm.armKinematics.getIKAnglesRadians());
        // cartesianStorage = new Translation3d(cartesianCoords.getX(), 0,
        // cartesianCoords.getZ());
    }
}
