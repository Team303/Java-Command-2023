package com.team303.robot.commands.claw;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;
import com.team303.robot.subsystems.ClawSubsystem.ClawState;
import com.team303.robot.subsystems.ClawSubsystem.GamePieceType;

import static com.team303.robot.Robot.claw;

public class DefaultClaw extends CommandBase {
    public DefaultClaw() {
        addRequirements(claw);
    }

    @Override
    public void execute() {
        // Based on the open/close state of the claw set by the operator, move the motor
        // in the correct direction
        if (claw.getState() == ClawState.OPEN) {
            // Only try to move the motor when the switch is not depressed
            if (!claw.outerLimitReached()) {
                claw.setClawSpeed(-0.5);
            } else {
                claw.setClawPosition(0);
            }
        } else {
            // Apply more force in cone mdoe and less in cube mode
            double pressure = claw.getMode() == GamePieceType.CONE ? 1 : 0.35;
            claw.setClawSpeed(pressure);
        }
    }
}
