package com.team303.robot.commands.arm;

import com.team303.robot.subsystems.ArmSubsystem;
import com.team303.robot.subsystems.PhotonvisionModule;
import com.team303.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReachGroundConeCommand extends CommandBase {
    private static final ArmSubsystem arm = ArmSubsystem.getArm();
    private static final SwerveSubsystem swerve = SwerveSubsystem.getSwerve();
    private static final PhotonvisionModule photonvision = PhotonvisionModule.getPhotonvision();
    
    public static PIDController xControl;
    public static PIDController yControl;

    public ReachGroundConeCommand() {
        addRequirements(swerve,arm);
        xControl = new PIDController(0.01,0,0);
        yControl = new PIDController(0.01,0,0);
    }

    public void execute() {
        if (photonvision.getPipelineIndex() != 1) {
            photonvision.setConePipeline();
        }
        //TODO: Find optimal distance for drivetrain from cone
        swerve.drive(
            new Translation2d(
            xControl.calculate(photonvision.getBestTarget().getBestCameraToTarget().getX(),Units.inchesToMeters(5)),
            yControl.calculate(photonvision.getBestTarget().getBestCameraToTarget().getY(),0)
            ),
            0,
            true
        );
        arm.reach(photonvision.getBestTarget().getBestCameraToTarget().getTranslation());
    }
    
}
