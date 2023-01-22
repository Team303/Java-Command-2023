package com.team303.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.team303.robot.Robot;
import com.team303.robot.RobotMap.PhotonvisionConstants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonvisionModule extends SubsystemBase {
    
    public static final ShuffleboardTab PHOTONVISION_TAB = Shuffleboard.getTab("PhotonVision");
    
    public static final NetworkTable photonvision = Robot.getNetworkTableInstance().getTable("PhotonVision"); 

    public static final GenericEntry APRILTAG_ID = PHOTONVISION_TAB.add("April ID", 0).getEntry();
    public static final GenericEntry TARGET_AMBIGUITY = PHOTONVISION_TAB.add("ID Ambiguity", 0).getEntry();
    public static final GenericEntry TARGET_YAW = PHOTONVISION_TAB.add("ID Yaw", 0).getEntry();
    public static final GenericEntry TARGET_PITCH = PHOTONVISION_TAB.add("ID Pitch", 0).getEntry();
    public static final GenericEntry TARGET_SKEW = PHOTONVISION_TAB.add("ID Skew", 0).getEntry();

    private final PhotonCamera camera;
    private static PhotonvisionModule instance = new PhotonvisionModule();
   // public static final GenericEntry TARGET_CORNERS = PHOTONVISION_TAB.add("ID Corners", 0).getEntry();

    private PhotonvisionModule() {
       camera = new PhotonCamera("photovision");
    }
    public static PhotonvisionModule getPhotonvision() {
        return instance;
    }
    public PhotonCamera getCamera() {
        return getPhotonvision().camera;
    }

    public PhotonPipelineResult getLatestResult() {
        return getPhotonvision().getCamera().getLatestResult();
    }

    public Boolean hasTargets() {
        return getPhotonvision().getLatestResult().hasTargets();
    }

    public List<PhotonTrackedTarget> getTargetList() {
        return getPhotonvision().getLatestResult().getTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        return getPhotonvision().getLatestResult().getBestTarget();
    }

    public void takeImage() {
        getPhotonvision().getCamera().takeInputSnapshot();
    }

    public void getImages() {
        getPhotonvision().getCamera().takeOutputSnapshot();
    }
    public void setCubePipeline() {
        getPhotonvision().getCamera().setPipelineIndex(0);
    }
    public void setConePipeline() {
        getPhotonvision().getCamera().setPipelineIndex(1);
    }
    public void setAprilTagPipeline() {
        getPhotonvision().getCamera().setPipelineIndex(2);
    }
    public int getPipelineIndex() {
        return getPhotonvision().getCamera().getPipelineIndex();
    }

    public double getDistanceToTarget() {
        int id = getBestTarget().getFiducialId();
        if (id != 4 || id != 5) {
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
        PhotonTrackedTarget target = getBestTarget();
        APRILTAG_ID.setInteger(target.getFiducialId());
        TARGET_AMBIGUITY.setDouble(target.getPoseAmbiguity());
        TARGET_YAW.setDouble(target.getYaw());
        TARGET_PITCH.setDouble(target.getPitch());
        TARGET_SKEW.setDouble(target.getSkew());
    }
}
