package com.team303.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static com.team303.robot.Robot.arm;
import com.team303.robot.RobotMap.Arm;

public class Homing extends CommandBase {

    double shoulderStartAngle;
    double elbowStartAngle;
    double clawStartAngle;
    double speed;

    public Homing() {
        addRequirements(arm);
    }

    @Override
    public void execute() {
        while (!arm.shoulderJoint.shoulderSwitch1.isPressed()) {
            arm.shoulderJoint.setMotors(-0.2);
        }   

        while(!arm.elbowJoint.elbowSwitch.isPressed()) {
            arm.elbowJoint.elbowMotor.set(-0.2);
        }
    }

    @Override
    public boolean isFinished() {
        return arm.shoulderJoint.shoulderSwitch1.isPressed() && arm.elbowJoint.elbowSwitch.isPressed();
    }

    @Override
    public void end(boolean interrupted) {

        shoulderStartAngle = (Math.toRadians(Math.round(-10)) / (Math.PI * 2)) * arm.shoulderJoint.shoulderEncoder1.getCountsPerRevolution() * Arm.GEAR_RATIO_SHOULDER;
		elbowStartAngle = (Math.toRadians(Math.round(170.0)) / (Math.PI * 2)) * arm.elbowJoint.elbowEncoder.getCountsPerRevolution() * Arm.GEAR_RATIO_ELBOW;

        arm.setEncoders(shoulderStartAngle, elbowStartAngle, 0);
    }
}
