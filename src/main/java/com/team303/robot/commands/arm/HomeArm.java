package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.commands.arm.DefaultIKControlCommand.cartesianStorage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import static com.team303.robot.subsystems.IntakeSubsystem.state;
import com.team303.robot.subsystems.IntakeSubsystem.IntakeState;

public class HomeArm extends CommandBase {
    /**
     * Deadline for the homing command to not delay the auto by more than is
     * necessary
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
        System.out.println(timer.get());
        arm.homeJoints();
    }

    @Override
    public boolean isFinished() {
        return arm.isInHomePosition() || timer.hasElapsed(HOMING_TIMEOUT);
    }

    @Override
    public void end(boolean interrupted) {
        // Reset arm encoders to known angles
        arm.stopMotors();
        arm.resetEncodersToHomePosition();

        // Reset the stored end effector point
        cartesianStorage = new Translation3d(13, 0, 20);

        // Stop the deadline timer
        timer.stop();
        state = IntakeState.NONE;
    }
}
