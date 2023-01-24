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
    public OpenClaw()
    {
    
    }

    @Override
    public void initialize() 
    {
        
	}

    @Override
    public void execute()
    {
        
    }
    @Override
    public boolean isFinished()
    {
        return true; 
    }


}
