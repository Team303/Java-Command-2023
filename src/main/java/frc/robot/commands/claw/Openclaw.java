package frc.robot.commands.claw;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClawSubsystem;

public class Openclaw extends CommandBase
{
    public Openclaw()
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
