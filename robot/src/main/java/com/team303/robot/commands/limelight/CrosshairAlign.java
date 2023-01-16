package com.team303.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import com.team303.robot.subsystems.LimelightModule;
import com.team303.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class CrosshairAlign extends CommandBase {

    public static final NetworkTable limelight = LimelightModule.getLimelight();
    public static final SwerveSubsystem swerve = SwerveSubsystem.getSwerve();

    public static PIDController xControl;
    public static PIDController yControl;


    public CrosshairAlign() {
        addRequirements(swerve);
        xControl = new PIDController(0.01, 0, 0);
        yControl = new PIDController(0.01, 0, 0);
    }

    @Override
    public void execute() {
        swerve.drive(
            new Translation2d(
                xControl.calculate(limelight.getEntry("tx").getDouble(0), 0),
                yControl.calculate(limelight.getEntry("ty").getDouble(0), 0)
            ), 
            0,
            true
        );
    }

    @Override
    public boolean isFinished() {
        return xControl.atSetpoint() && yControl.atSetpoint();
    }
}
