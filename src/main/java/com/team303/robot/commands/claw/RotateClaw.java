/*package com.team303.robot.commands.claw;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;
import com.team303.robot.RobotMap;
import com.team303.robot.subsystems.ClawSubsystem;

public class RotateClaw extends CommandBase {
    
    private static double rotateLimit;
    private static double speed;

    public RotateClaw(double r, double s) {
        addRequirements(claw);
        rotateLimit = r;
        speed = s;
    }

    @Override
    public void end(boolean inerrupted) 
    {
        claw.claw(0.0);
        claw.resetEncoders();
	}

    @Override
    public void execute()
    {
        claw.rotate(speed);
    }
    @Override
    public boolean isFinished()
    {
        return claw.getRotate(rotateLimit); 
    }




}*/