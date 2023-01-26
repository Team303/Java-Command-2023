package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.Robot.photonvision;
import static com.team303.robot.Robot.poseTracker;
import static com.team303.robot.Robot.swerve;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.team303.robot.modules.Photonvision.PhotonPipeline;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;;

public class ReachCubeToNodeCommand extends CommandBase {
    
    public static PIDController xControl;
    public static PIDController yControl;

    public ReachCubeToNodeCommand() {
        addRequirements(swerve, arm);
        xControl = new PIDController(0.01, 0, 0);
        yControl = new PIDController(0.01, 0, 0);
    }

    @Override
    public void execute() {
        if (photonvision.getPipeline() != PhotonPipeline.APRILTAG) {
            photonvision.setPipeline(PhotonPipeline.APRILTAG);
        }
        PhotonTrackedTarget target = photonvision.getBestTarget();
        // TODO: Find optimal distance from drivetrain to node
        swerve.drive(
                new Translation2d(
                        xControl.calculate(target.getBestCameraToTarget().getX(), Units.inchesToMeters(3)),
                        yControl.calculate(target.getBestCameraToTarget().getY(), 0)),
                0,
                true);
        Translation3d armToAprilTag = poseTracker.getArmtoTargetTranslation();
        photonvision.setPipeline(PhotonPipeline.CUBE);
        // TODO: Find optimal high row pitch angle threshold to check for
        // already-present cubes
        if (photonvision.hasTargets() && photonvision.getBestTarget().getPitch() >= 60) {
            // Reach mid row
            arm.reach(armToAprilTag
                    .plus(new Translation3d(Units.inchesToMeters(15.8125), 0.0, Units.inchesToMeters(20.25))));
        } else {
            // Reach high row
            arm.reach(armToAprilTag
                    .plus(new Translation3d(Units.inchesToMeters(47.4375), 0.0, Units.inchesToMeters(32.25))));
        }
    }
}
