package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.commands.arm.DefaultIKControlCommand.cartesianStorage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation3d;

public class HomeArmContinuous extends CommandBase {

    double shoulderSpeed;
    double elbowSpeed;
    double wristSpeed;

    public HomeArmContinuous(double shoulderSpeed, double elbowSpeed, double wristSpeed) {
        addRequirements(arm);
        this.shoulderSpeed = shoulderSpeed;
        this.elbowSpeed = elbowSpeed;
        this.wristSpeed = wristSpeed;
    }

    @Override
    public void execute() {
        arm.homeJoints(this.shoulderSpeed, this.elbowSpeed, this.wristSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Reset arm encoders to known angles
        arm.resetEncodersToHomePosition();

        cartesianStorage = new Translation3d(10, 0, 17);
    }
}
