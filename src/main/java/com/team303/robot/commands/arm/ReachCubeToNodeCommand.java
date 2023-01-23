package com.team303.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.team303.robot.subsystems.ArmSubsystem;
import com.team303.robot.subsystems.SwerveSubsystem;
import com.team303.robot.subsystems.PhotonvisionModule;
import com.team303.robot.subsystems.PhotonvisionModule.PhotonPipeline;;

public class ReachCubeToNodeCommand extends CommandBase {
    private static final ArmSubsystem arm = ArmSubsystem.getArm();
    private static final SwerveSubsystem swerve = SwerveSubsystem.getSwerve();
    private static final PhotonvisionModule photonvision = PhotonvisionModule.getPhotonvision();
    
    public static PIDController xControl;
    public static PIDController yControl;
    
    public ReachCubeToNodeCommand() {
        addRequirements(swerve,arm);
        xControl = new PIDController(0.01,0,0);
        yControl = new PIDController(0.01,0,0);
    }

    @Override
    public void execute() {
        if (photonvision.getPipeline() != PhotonPipeline.APRILTAG) {
            photonvision.setPipeline(PhotonPipeline.APRILTAG);
        }
        //TODO: Find optimal distance from drivetrain to node
        swerve.drive(
            new Translation2d(
            xControl.calculate(photonvision.getBestTarget().getBestCameraToTarget().getX(),Units.inchesToMeters(3)),
            yControl.calculate(photonvision.getBestTarget().getBestCameraToTarget().getY(),0)
            ),
            0,
            true
        );
        photonvision.setPipeline(PhotonPipeline.CUBE);
        //TODO: Find optimal joint angles
        if (photonvision.hasTargets()) {
            //Reach mid row
            arm.reach(new double[]{Math.PI/4,Math.PI/4,Math.PI/4});
        } else {
            //Reach high row
            arm.reach(new double[]{Math.PI/4,Math.PI/4,Math.PI/4});
        }
    }
}
