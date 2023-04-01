package com.team303.robot.commands.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;
import com.team303.robot.subsystems.IntakeSubsystem.IntakeState;
import com.team303.robot.subsystems.ManipulatorSubsystem.GamePieceType;

import static com.team303.robot.Robot.intake;

public class DefaultIntake extends CommandBase {
    public DefaultIntake() {
        addRequirements(intake);
    }

    @Override
    public void execute() {
        // Based on the open/close state of the intake set by the operator, move the motor
        // in the correct direction
        //TODO: Check direction of motor
        if (intake.getState() == IntakeState.INTAKE) {
            // Only try to move the motor when the switch is not depressed
                intake.setManipulatorSpeed(-0.5);
        } else if (intake.getState() == IntakeState.OUTTAKE) {
            intake.setManipulatorSpeed(0.5);
        } else {
            intake.setManipulatorSpeed(0);
        }

    }
}
