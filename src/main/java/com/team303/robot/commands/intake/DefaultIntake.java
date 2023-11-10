package com.team303.robot.commands.intake;

import static com.team303.robot.Robot.intake;

import com.team303.robot.subsystems.IntakeSubsystem.IntakeState;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultIntake extends CommandBase {
    public DefaultIntake() {
        addRequirements(intake);
    }

    @Override
    public void execute() {
        // Based on the open/close state of the claw set by the operator, move the motor
        // in the correct direction
        // TODO: Check direction of motor
        if (intake.getState() == IntakeState.INTAKE) {
            // Only try to move the motor when the switch is not depressed
            intake.setManipulatorSpeed(-1);
        } else if (intake.getState() == IntakeState.OUTTAKE) {
            intake.setManipulatorSpeed(1);
        } else {
            intake.setManipulatorSpeed(0);
        }


    }
}
