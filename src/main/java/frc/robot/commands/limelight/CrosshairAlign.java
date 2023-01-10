package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class CrosshairAlign extends CommandBase {

    public static final NetworkTable limelight = LimelightSubsystem.getLimelight();
    public static final SwerveSubsystem swerve = SwerveSubsystem.getSwerve();

    public static PIDController xControl;
    public static PIDController yControl;


    public CrosshairAlign() {
        addRequirements(swerve);
        PIDController xControl = new PIDController(0.01, 0, 0);
        PIDController yControl = new PIDController(0.01, 0, 0);
    }

    @Override
    public void execute() {
        swerve.fieldoOrientedDrive(
            new Translation2d(
                xControl.calculate(limelight.getEntry("tx").getDouble(0), 0),
                yControl.calculate(limelight.getEntry("ty").getDouble(0), 0)
            ), 
            0
        );
    }

    @Override
    public boolean isFinished() {
        return xControl.atSetpoint() && yControl.atSetpoint();
    }
}
