package com.team303.robot.commands.arm;

import com.team303.robot.Robot;
import com.team303.robot.subsystems.ArmSubsystem;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultIKControlCommand extends CommandBase {
        public DefaultIKControlCommand() {
        addRequirements(ArmSubsystem.getArm());
    }
    public void execute() {
        ArmSubsystem.getArm().reach(
            new Translation3d(
                Robot.getXbox().getLeftX(), 
                0.0,
                Robot.getXbox().getLeftY()
            )
        );

    }

    
}
