package com.team303.robot.commands.arm;

import com.team303.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultIKControlCommand extends CommandBase {
        public DefaultIKControlCommand() {
        addRequirements(ArmSubsystem.getArm());
    }

    
}
