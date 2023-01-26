package com.team303.robot.commands.claw;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;
import com.team303.robot.RobotMap;
import com.team303.robot.subsystems.ClawSubsystem;
public class CloseClaw extends CommandBase
{
    public CloseClaw()
    {
    addRequirements(ClawSubsystem.getClaw());
    } 

    @Override
    public void execute()
    {
        ClawSubsystem.getClaw().claw(1.0);
    }
    @Override
    public boolean isFinished()
    {
        return ClawSubsystem.getClaw().innerLimitReached(); 
    }
    @Override
    public void end(boolean interrupted)
    {
        ClawSubsystem.getClaw().claw(0);
    }


}
