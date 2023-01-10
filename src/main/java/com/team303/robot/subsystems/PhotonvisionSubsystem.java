package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.networktables.NetworkTable;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.List;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotMap.PhotonvisionConstants;

public class PhotonvisionSubsystem extends SubsystemBase {
    
    public static final ShuffleboardTab PHOTONVISION_TAB = Shuffleboard.getTab("PhotonVision");
    
    public static final GenericEntry APRILTAG_ID = PHOTONVISION_TAB.add("April ID", 0).getEntry();
    public static final GenericEntry TARGET_AMBIGUITY = PHOTONVISION_TAB.add("ID Ambiguity", 0).getEntry();
    public static final GenericEntry TARGET_YAW = PHOTONVISION_TAB.add("ID Yaw", 0).getEntry();
    public static final GenericEntry TARGET_PITCH = PHOTONVISION_TAB.add("ID Pitch", 0).getEntry();
    public static final GenericEntry TARGET_SKEW = PHOTONVISION_TAB.add("ID Skew", 0).getEntry();
   // public static final GenericEntry TARGET_CORNERS = PHOTONVISION_TAB.add("ID Corners", 0).getEntry();

    private static PhotonCamera camera = new PhotonCamera("photovision");

    public PhotonCamera getCamera() {
        return camera;
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public Boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    public List<PhotonTrackedTarget> getTargetList() {
        return camera.getLatestResult().getTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        return camera.getLatestResult().getBestTarget();
    }

    public void takeImage() {
        camera.takeInputSnapshot();
    }

    public void getImages() {
        camera.takeOutputSnapshot();
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
