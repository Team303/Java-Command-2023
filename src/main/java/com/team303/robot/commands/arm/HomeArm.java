package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.commands.arm.DefaultIKControlCommand.cartesianStorage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation3d;

public class HomeArm extends CommandBase {
    /**
     * Deadline for the homing command to not delay the auto by more than is necessary
     */
    private static final double HOMING_TIMEOUT = 3;

    /**
     * Deadline timer to end the command if it takes to long
     */
    Timer timer = new Timer();

    public HomeArm() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        arm.homeJoints();
    }

    @Override
    public boolean isFinished() {
        return arm.isInHomePosition() || timer.hasElapsed(HOMING_TIMEOUT);
    }

    @Override
    public void end(boolean interrupted) {
        // Reset arm encoders to known angles
        arm.resetEncodersToHomePosition();

        // Reset the stored end effector point
        cartesianStorage = new Translation3d(8, 0, 15);

        // Stop the deadline timer
        timer.stop();
    }
}