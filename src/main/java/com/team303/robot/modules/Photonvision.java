package com.team303.robot.modules;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.team303.robot.Robot;
import com.team303.robot.RobotMap.PhotonvisionConstants;
import static com.team303.robot.Robot.ALLIANCE_SUBSTATION_ID;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Photonvision extends SubsystemBase {

    public static final ShuffleboardTab PHOTONVISION_TAB = Shuffleboard.getTab("PhotonVision");

    public static final NetworkTable photonvision = NetworkTableInstance.getDefault().getTable("PhotonVision");

    public static final GenericEntry APRILTAG_ID = PHOTONVISION_TAB.add("April ID", 0).getEntry();
    public static final GenericEntry TARGET_AMBIGUITY = PHOTONVISION_TAB.add("ID Ambiguity", 0).getEntry();
    public static final GenericEntry TARGET_YAW = PHOTONVISION_TAB.add("ID Yaw", 0).getEntry();
    public static final GenericEntry TARGET_PITCH = PHOTONVISION_TAB.add("ID Pitch", 0).getEntry();
    public static final GenericEntry TARGET_SKEW = PHOTONVISION_TAB.add("ID Skew", 0).getEntry();

    private static PhotonCamera camera = new PhotonCamera("photovision");

    public static PhotonTrackedTarget target;

    public enum PhotonPipeline {
        CUBE,
        CONE,
        APRILTAG;
    }

    public static PhotonCamera getCamera() {
        return camera;
    }

    public static PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public static Boolean hasTargets() {
        return getLatestResult().hasTargets();
    }

    public static List<PhotonTrackedTarget> getTargetList() {
        return getLatestResult().getTargets();
    }

    public static PhotonTrackedTarget getBestTarget() {
        if (hasTargets()) {
            return getLatestResult().getBestTarget();
        }
        return null;
    }

    public static void takeImage() {
        getCamera().takeInputSnapshot();
    }

    public static void getImages() {
        camera.takeOutputSnapshot();
    }

    public static void setPipeline(PhotonPipeline pipelineName) {
        camera.setPipelineIndex(pipelineName.ordinal());
    }

    public static PhotonPipeline getPipeline() {
        return PhotonPipeline.values()[camera.getPipelineIndex()];
    }

    public static double getDistanceToTarget() {

        if (!hasTargets()) {
            return Double.NaN;
        } 

        int id = getBestTarget().getFiducialId();
        if (id != ALLIANCE_SUBSTATION_ID) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                    PhotonvisionConstants.CAMERA_HEIGHT_METERS,
                    PhotonvisionConstants.GRID_TARGET_HEIGHT_METERS,
                    PhotonvisionConstants.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(getBestTarget().getPitch()));
        } else {
            return PhotonUtils.calculateDistanceToTargetMeters(
                    PhotonvisionConstants.CAMERA_HEIGHT_METERS,
                    PhotonvisionConstants.DOUBLE_SUBSTATION_TARGET_HEIGHT_METERS,
                    PhotonvisionConstants.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(getBestTarget().getPitch()));
        }
    }

    @Override
    public void periodic() {
        target = getBestTarget();

        if (target == null) {
            return;
        }
        
        APRILTAG_ID.setInteger(target.getFiducialId());
        TARGET_AMBIGUITY.setDouble(target.getPoseAmbiguity());
        TARGET_YAW.setDouble(target.getYaw());
        TARGET_PITCH.setDouble(target.getPitch());
        TARGET_SKEW.setDouble(target.getSkew());
    }
}
