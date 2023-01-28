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

    private static PhotonCamera[] camera = {
        new PhotonCamera("photonvision"), 
        new PhotonCamera("photonvision2")};

    public static PhotonTrackedTarget target;

    public static enum PhotonPipeline {
        CUBE,
        CONE,
        APRILTAG;
    }

    public static enum CameraName {
        CAM1,
        CAM2
    }

    public PhotonCamera getCamera(CameraName name) {
        return camera[name.ordinal()];
    }

    public PhotonPipelineResult getLatestResult(CameraName name) {
        return camera[name.ordinal()].getLatestResult();
    }

    public Boolean hasTargets(CameraName name) {
        return getLatestResult(name).hasTargets();
    }

    public List<PhotonTrackedTarget> getTargetList(CameraName name) {
        return getLatestResult(name).getTargets();
    }

    public PhotonTrackedTarget getBestTarget(CameraName name) {
        if (hasTargets(name)) {
            return getLatestResult(name).getBestTarget();
        }
        return null;
    }

    public void takeImage(CameraName name) {
        getCamera(name).takeInputSnapshot();
    }

    public void getImages(CameraName name) {
        getCamera(name).takeOutputSnapshot();
    }

    public void setPipeline(CameraName name, PhotonPipeline pipelineName) {
        getCamera(name).setPipelineIndex(pipelineName.ordinal());
    }

    public PhotonPipeline getPipeline(CameraName name) {
        return PhotonPipeline.values()[getCamera(name).getPipelineIndex()];
    }

    public double getDistanceToTarget() {

        if (!hasTargets(CameraName.CAM1)) {
            return Double.NaN;
        } 

        int id = getBestTarget(CameraName.CAM1).getFiducialId();
        if (id != ALLIANCE_SUBSTATION_ID) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                    PhotonvisionConstants.CAMERA_HEIGHT_METERS,
                    PhotonvisionConstants.GRID_TARGET_HEIGHT_METERS,
                    PhotonvisionConstants.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(getBestTarget(CameraName.CAM1).getPitch()));
        } else {
            return PhotonUtils.calculateDistanceToTargetMeters(
                    PhotonvisionConstants.CAMERA_HEIGHT_METERS,
                    PhotonvisionConstants.DOUBLE_SUBSTATION_TARGET_HEIGHT_METERS,
                    PhotonvisionConstants.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(getBestTarget(CameraName.CAM1).getPitch()));
        }
    }

    @Override
    public void periodic() {
        if (getBestTarget(CameraName.CAM1) == null) {
            return;
        }
        
        APRILTAG_ID.setInteger(target.getFiducialId());
        TARGET_AMBIGUITY.setDouble(target.getPoseAmbiguity());
        TARGET_YAW.setDouble(target.getYaw());
        TARGET_PITCH.setDouble(target.getPitch());
        TARGET_SKEW.setDouble(target.getSkew());
    }
}
