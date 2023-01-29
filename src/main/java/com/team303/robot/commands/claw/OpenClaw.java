package com.team303.robot.commands.claw;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;
import com.team303.robot.RobotMap;
import com.team303.robot.subsystems.ClawSubsystem;

public class OpenClaw extends CommandBase
{

    private static double encoderPos;

    public OpenClaw(double e) {
        addRequirements(Robot.claw);
        encoderPos = e;
    }

    @Override
    public void end(boolean inerrupted) 
    {
        Robot.claw.claw(0.0);
        Robot.claw.resetEncoders();
	}

    @Override
    public void execute() {
        Robot.claw.claw(-1);
    }

    @Override
    public boolean isFinished() {
        
        return Robot.claw.outerLimitReached() || Robot.claw.getEncoderPos(encoderPos); 
    }
}
