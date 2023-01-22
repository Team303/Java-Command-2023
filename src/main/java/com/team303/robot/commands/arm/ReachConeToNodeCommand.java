package com.team303.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.team303.robot.subsystems.ArmSubsystem;
import com.team303.robot.subsystems.LimelightModule;
import com.team303.robot.subsystems.SwerveSubsystem;

//Make sure to run CrosshairAlign before this command!
public class ReachConeToNodeCommand extends CommandBase {
    private static final ArmSubsystem arm = ArmSubsystem.getArm();
    private static final SwerveSubsystem swerve = SwerveSubsystem.getSwerve();
    private static final NetworkTable limelight = LimelightModule.getLimelight();

    public static PIDController xControl;

    public ReachConeToNodeCommand() {
        addRequirements(swerve,arm);
        xControl = new PIDController(0.01,0,0);
        
       
    }

    @Override
    public void execute() {
        swerve.drive(
            new Translation2d(
            xControl.calculate(limelight.getEntry("ta").getDouble(0), 0.125),
            0
            ),
            0,
            true
        );
        arm.reach(new double[]{Math.PI/4,Math.PI/4,Math.PI/4});
        
    }
    
}
