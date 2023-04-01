package com.team303.robot.commands.intake;

import static com.team303.robot.Robot.manipulator;

import com.team303.robot.subsystems.IntakeSubsystem.IntakeState;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultIntake extends CommandBase {
    public DefaultIntake() {
        addRequirements(manipulator);
    }

    @Override
    public void execute() {
        // Based on the open/close state of the claw set by the operator, move the motor
        // in the correct direction
        // TODO: Check direction of motor
        if (manipulator.getState() == IntakeState.INTAKE) {
            // Only try to move the motor when the switch is not depressed
            manipulator.setManipulatorSpeed(-0.5);
        } else if (manipulator.getState() == IntakeState.OUTTAKE) {
            manipulator.setManipulatorSpeed(0.5);
        } else {
            manipulator.setManipulatorSpeed(0);
        }

    }
}
