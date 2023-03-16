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
        if (!arm.shoulderJoint.shoulderSwitchReverse1.isPressed()) {
            arm.shoulderJoint.setMotors(-0.05);
        }   

        if (!arm.elbowJoint.elbowSwitchReverse.isPressed()) {
            arm.elbowJoint.elbowMotor.set(0.05);
        }
    }

    @Override
    public boolean isFinished() {
        return arm.shoulderJoint.shoulderSwitchReverse1.isPressed() && arm.elbowJoint.elbowSwitchReverse.isPressed();
    }

    @Override
    public void end(boolean interrupted) {

        shoulderStartAngle = (Math.toRadians(Math.round(-20)) / (Math.PI * 2)) * arm.shoulderJoint.shoulderEncoder1.getCountsPerRevolution();
		elbowStartAngle = (Math.toRadians(Math.round(170.0)) / (Math.PI * 2)) * arm.elbowJoint.elbowEncoder.getCountsPerRevolution();
        
        arm.setEncoders(shoulderStartAngle, elbowStartAngle, 0);
    }
}
