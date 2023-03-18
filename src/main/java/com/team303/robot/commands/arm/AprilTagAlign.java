package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.photonvision;
import static com.team303.robot.Robot.swerve;
import static com.team303.robot.subsystems.SwerveSubsystem.DIFFERENCE;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.team303.robot.modules.Photonvision.CameraName;
import com.team303.robot.modules.Photonvision.PhotonPipeline;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AprilTagAlign extends CommandBase {

    public static PIDController xControl;
    public static PIDController yControl;
    public static PIDController yawControl;
    private final double alignOffset;
    PhotonTrackedTarget target;

    public AprilTagAlign(int node) {

        switch (node % 3) {
            case 0:
                alignOffset = -DIFFERENCE;
                break; // left
            case 2:
                alignOffset = DIFFERENCE;
                break; // right
            default:
                alignOffset = 0;
                break; // middle
        }

        // addRequirements(swerve, arm, photonvision);
        addRequirements(swerve, photonvision);
        xControl = new PIDController(0.15, 0, 0.02);
        yControl = new PIDController(0.03, 0, 0.01);
        yawControl = new PIDController(0.1, 0, 0);
        xControl.setTolerance(0.1);
        yControl.setTolerance(0.8);
        yawControl.setTolerance(1);
        target = photonvision.getBestTarget(CameraName.CAM1);

        if (photonvision.getPipeline(CameraName.CAM1) != PhotonPipeline.AprilTag) {
            photonvision.setPipeline(CameraName.CAM1, PhotonPipeline.AprilTag);
        }
    }

    @Override
    public void execute() {
        if (photonvision.hasTargets(CameraName.CAM1)) {
            PhotonTrackedTarget target = photonvision.getBestTarget(CameraName.CAM1);

            if (target == null) {
                return;
            }

            try {
                swerve.drive(
                        new Translation2d(
                                -xControl.calculate((target.getBestCameraToTarget().getY() + alignOffset) * 5, 0),
                                yControl.calculate(target.getBestCameraToTarget().getX() * 10, 0)),
                        // yawControl.calculate(target.getYaw(), 0),
                        0,
                        true);

                System.out.println("robot x PID: " + -xControl.calculate(target.getBestCameraToTarget().getY() * 5, 0));
                System.out.println("robot y: " + target.getBestCameraToTarget().getX());
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public boolean isFinished() {

        return target != null && Math.abs(target.getBestCameraToTarget().getX()) < 0.85;
    }
}
