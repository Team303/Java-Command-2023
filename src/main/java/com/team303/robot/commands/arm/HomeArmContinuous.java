package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.commands.arm.DefaultIKControlCommand.cartesianStorage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation3d;

public class HomeArmContinuous extends CommandBase {
    public HomeArmContinuous() {
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.homeJoints();
    }

    @Override
    public void end(boolean interrupted) {
        // Reset arm encoders to known angles
        arm.resetEncodersToHomePosition();

        cartesianStorage = new Translation3d(8, 0, 15);
    }
}
