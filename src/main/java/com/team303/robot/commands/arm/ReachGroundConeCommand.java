package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.Robot.photonvision;
import static com.team303.robot.Robot.poseTracker;
import static com.team303.robot.Robot.swerve;

import com.team303.robot.modules.Photonvision.PhotonPipeline;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.modules.Photonvision.CameraName;

public class ReachGroundConeCommand extends CommandBase {

    public static PIDController xControl;
    public static PIDController yControl;

    public ReachGroundConeCommand() {
        addRequirements(swerve, arm);
        xControl = new PIDController(0.01, 0, 0);
        yControl = new PIDController(0.01, 0, 0);
    }

    @Override
    public void execute() {
        if (photonvision.getPipeline(CameraName.CAM1) != PhotonPipeline.CONE) {
            photonvision.setPipeline(CameraName.CAM1, PhotonPipeline.CONE);
        }
        // TODO: Find optimal distance for drivetrain from cone
        swerve.drive(
                new Translation2d(
                        xControl.calculate(photonvision.getBestTarget(CameraName.CAM1).getBestCameraToTarget().getX(),
                                Units.inchesToMeters(5)),
                        yControl.calculate(photonvision.getBestTarget(CameraName.CAM1).getBestCameraToTarget().getY(), 0)),
                0,
                true);
        Translation3d armToCone = poseTracker.getArmtoTargetTranslation();
        // TODO: Find optimal part of cone to grab
        arm.reach(armToCone.plus(new Translation3d()));
    }

}
