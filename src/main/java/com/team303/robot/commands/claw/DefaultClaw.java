package com.team303.robot.commands.claw;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;
import com.team303.robot.subsystems.ClawSubsystem;
import com.team303.robot.subsystems.ClawSubsystem.ClawState;
import com.team303.robot.subsystems.ManipulatorSubsystem.GamePieceType;

import static com.team303.robot.Robot.manipulator;

public class DefaultClaw extends CommandBase {
    public DefaultClaw() {
        addRequirements(manipulator);
    }

    @Override
    public void execute() {
        // Based on the open/close state of the claw set by the operator, move the motor
        // in the correct direction
        if (manipulator instanceof ClawSubsystem) {
            ClawSubsystem claw = (ClawSubsystem) manipulator;
        if (manipulator.getState() == ClawState.OPEN) {
            // Only try to move the motor when the switch is not depressed
            if (!claw.outerLimitReached()) {
                claw.setManipulatorSpeed(-0.5);
            } else {
                claw.setManipulatorPosition(0);
            }
        } else {
            // Apply more force in cone mdoe and less in cube mode
            double pressure = manipulator.getMode() == GamePieceType.CONE ? 1 : 0.35;
            manipulator.setManipulatorSpeed(pressure);
        }
    } else {

    }
    }
}
