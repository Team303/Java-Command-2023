package com.team303.robot.commands.arm;

import com.team303.robot.Robot;
import static com.team303.robot.RobotMap.IOConstants.DEADBAND_FILTER;
import com.team303.robot.subsystems.ArmSubsystem;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultIKControlCommand extends CommandBase {
        public DefaultIKControlCommand() {
        addRequirements(ArmSubsystem.getArm());
    }

    @Override
    public void execute() {
        ArmSubsystem.getArm().reach(
            new Translation3d(
                DEADBAND_FILTER.applyDeadband(Robot.getXbox().getLeftX()), 
                0.0,
                DEADBAND_FILTER.applyDeadband(Robot.getXbox().getLeftY())
            )
        );

    }

    
}
