package com.team303.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;

public class ToggleCloseCube extends CommandBase {

    public static final int ANGLE = 45;

    public ToggleCloseCube() {
        addRequirements(Robot.claw);
    }

    @Override
    public void initialize() {
        Robot.claw.setClawPosition(0);
    }

    @Override
    public void execute() {
        Robot.claw.setClawSpeed(-0.2);
    }

    @Override
    public boolean isFinished() {
        return Robot.claw.getClawPosition() > ANGLE;
    }
}
