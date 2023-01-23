package com.team303.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.team303.robot.subsystems.ArmSubsystem;
import com.team303.robot.subsystems.SwerveSubsystem;
import com.team303.robot.subsystems.PhotonvisionModule;
import com.team303.robot.subsystems.PoseEstimatorModule;
import com.team303.robot.subsystems.PhotonvisionModule.PhotonPipeline;;

public class ReachCubeToNodeCommand extends CommandBase {
    private static final ArmSubsystem arm = ArmSubsystem.getArm();
    private static final SwerveSubsystem swerve = SwerveSubsystem.getSwerve();
    private static final PhotonvisionModule photonvision = PhotonvisionModule.getPhotonvision();
    private static final PoseEstimatorModule poseEstimator = PoseEstimatorModule.getPoseSubsystem();

    
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
        PhotonTrackedTarget target = photonvision.getBestTarget();
        //TODO: Find optimal distance from drivetrain to node
        swerve.drive(
            new Translation2d(
            xControl.calculate(target.getBestCameraToTarget().getX(),Units.inchesToMeters(3)),
            yControl.calculate(target.getBestCameraToTarget().getY(),0)
            ),
            0,
            true
        );
        Translation3d armToAprilTag = poseEstimator.getArmtoTargetTranslation(PhotonPipeline.APRILTAG);
        photonvision.setPipeline(PhotonPipeline.CUBE);
        //TODO: Find optimal high row pitch angle threshold to check for already-present cubes
        if (photonvision.hasTargets() && photonvision.getBestTarget().getPitch() >= 60) {
            //Reach mid row
            arm.reach(armToAprilTag.plus(new Translation3d(Units.inchesToMeters(15.8125),0.0,Units.inchesToMeters(20.25))));
        } else {
            //Reach high row
            arm.reach(armToAprilTag.plus(new Translation3d(Units.inchesToMeters(47.4375),0.0,Units.inchesToMeters(32.25))));
        }
    }
}
