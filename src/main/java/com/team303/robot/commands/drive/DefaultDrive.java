package com.team303.robot.commands.drive;

import static com.team303.robot.Robot.ALLIANCE_SUBSTATION_ID;
import static com.team303.robot.Robot.arm;
import static com.team303.robot.Robot.photonvision;
import static com.team303.robot.Robot.poseTracker;
import static com.team303.robot.Robot.swerve;
import static com.team303.robot.RobotMap.IOConstants.DEADBAND_FILTER;

import com.team303.robot.Robot;
import com.team303.robot.RobotMap.Swerve;
import com.team303.robot.modules.Photonvision.PhotonPipeline;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultDrive extends CommandBase {

    boolean fieldOriented;

    public DefaultDrive(boolean fieldOriented) {
        addRequirements(swerve);
        this.fieldOriented = fieldOriented;
    }

    @Override
    public void execute() {
        swerve.drive(
                new Translation2d(
                        DEADBAND_FILTER.applyDeadband(Robot.getRightJoyStick().getX(), DEADBAND_FILTER.getLowerBound())
                                * Swerve.MAX_VELOCITY,
                        DEADBAND_FILTER.applyDeadband(Robot.getRightJoyStick().getY(), DEADBAND_FILTER.getLowerBound())
                                * Swerve.MAX_VELOCITY),
                DEADBAND_FILTER.applyDeadband(Robot.getLeftJoyStick().getY(), DEADBAND_FILTER.getLowerBound()),
                fieldOriented);
    if (photonvision.getPipeline() != PhotonPipeline.APRILTAG) {
        photonvision.setPipeline(PhotonPipeline.APRILTAG);
    }
    //TODO: Find good area threshold
    if (photonvision.getBestTarget().getFiducialId() == ALLIANCE_SUBSTATION_ID && photonvision.getBestTarget().getArea() >= 1) {
        photonvision.setPipeline(PhotonPipeline.CUBE);
        if (!photonvision.hasTargets()) {
            photonvision.setPipeline(PhotonPipeline.CONE);
        }
        Translation3d armToPiece = poseTracker.getArmtoTargetTranslation();
        //TODO: Find optimal part of cone to grab
        arm.reach(armToPiece.plus(new Translation3d()));
    //FIXME: Change when we introduce more cameras
    //FIXME: Change when we create autonomous driving during teleop and move it to the autonomous part
    }
        
    }
}
