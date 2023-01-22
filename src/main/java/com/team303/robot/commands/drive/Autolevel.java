package com.team303.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import com.team303.robot.Robot;
import com.team303.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class Autolevel extends PIDCommand{

    private static final PIDController PID_CONTROLLER = new PIDController(0.01, 0, 0);

    public Autolevel(double gyro)
    {
        super(
            PID_CONTROLLER,
            () -> -Robot.getNavX().getRawGyroY(),
            gyro,
            (output) -> SwerveSubsystem.getSwerve().drive(new Translation2d(0, output), 0.0,true),
            SwerveSubsystem.getSwerve());

            getController().setTolerance(1, 0);

    }

    @Override
    public boolean isFinished()
    {
        return getController().atSetpoint();
    }

}
