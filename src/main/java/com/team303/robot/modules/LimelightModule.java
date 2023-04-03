package com.team303.robot.modules;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightModule extends SubsystemBase {

    /* ShuffleBoard */
    public static final ShuffleboardTab LIMELIGHT_TAB = Shuffleboard.getTab("limelightLeft");

    public static final NetworkTable limelightLeft = NetworkTableInstance.getDefault().getTable("limelightLeft");
    public static final NetworkTable limelightRight = NetworkTableInstance.getDefault().getTable("limelightRight");

    public static final DoubleSubscriber VALID_TARGETS_SUB_LEFT = limelightLeft.getDoubleTopic("tv").subscribe(0);
    public static final DoubleSubscriber OFFSET_X_SUB_LEFT = limelightLeft.getDoubleTopic("tx").subscribe(0.0);
    public static final DoubleSubscriber OFFSET_Y_SUB_LEFT = limelightLeft.getDoubleTopic("ty").subscribe(0.0);
    public static final DoubleSubscriber TARGET_AREA_SUB_LEFT = limelightLeft.getDoubleTopic("ta").subscribe(0.0);
    public static final DoubleSubscriber SKEW_ROTATION_SUB_LEFT = limelightLeft.getDoubleTopic("ts").subscribe(0.0);
    public static final DoubleSubscriber APRILTAG_ID_SUB_LEFT = limelightLeft.getDoubleTopic("tid").subscribe(-1.0);

    public static final DoublePublisher VALID_TARGETS_PUB_LEFT = limelightLeft.getDoubleTopic("Valid Targets Out")
            .publish();
    public static final DoublePublisher OFFSET_X_PUB_LEFT = limelightLeft.getDoubleTopic("Offset X Out").publish();
    public static final DoublePublisher OFFSET_Y_PUB_LEFT = limelightLeft.getDoubleTopic("Offset Y Out").publish();
    public static final DoublePublisher TARGET_AREA_PUB_LEFT = limelightLeft.getDoubleTopic("Target Area Out")
            .publish();
    public static final DoublePublisher SKEW_ROTATION_PUB_LEFT = limelightLeft.getDoubleTopic("Skew Rotation Out")
            .publish();
    public static final DoublePublisher APRILTAG_ID_PUB_LEFT = limelightLeft.getDoubleTopic("Fiducial ID Out")
            .publish();

    public static final DoubleSubscriber VALID_TARGETS_SUB_RIGHT = limelightRight.getDoubleTopic("tv").subscribe(0);
    public static final DoubleSubscriber OFFSET_X_SUB_RIGHT = limelightRight.getDoubleTopic("tx").subscribe(0.0);
    public static final DoubleSubscriber OFFSET_Y_SUB_RIGHT = limelightRight.getDoubleTopic("ty").subscribe(0.0);
    public static final DoubleSubscriber TARGET_AREA_SUB_RIGHT = limelightRight.getDoubleTopic("ta").subscribe(0.0);
    public static final DoubleSubscriber SKEW_ROTATION_SUB_RIGHT = limelightRight.getDoubleTopic("ts").subscribe(0.0);
    public static final DoubleSubscriber APRILTAG_ID_SUB_RIGHT = limelightRight.getDoubleTopic("tid").subscribe(-1.0);

    public static final DoublePublisher VALID_TARGETS_PUB_RIGHT = limelightRight.getDoubleTopic("Valid Targets Out")
            .publish();
    public static final DoublePublisher OFFSET_X_PUB_RIGHT = limelightRight.getDoubleTopic("Offset X Out").publish();
    public static final DoublePublisher OFFSET_Y_PUB_RIGHT = limelightRight.getDoubleTopic("Offset Y Out").publish();
    public static final DoublePublisher TARGET_AREA_PUB_RIGHT = limelightRight.getDoubleTopic("Target Area Out")
            .publish();
    public static final DoublePublisher SKEW_ROTATION_PUB_RIGHT = limelightRight.getDoubleTopic("Skew Rotation Out")
            .publish();
    public static final DoublePublisher APRILTAG_ID_PUB_RIGHT = limelightRight.getDoubleTopic("tid").publish();

    public static enum CameraName {
        CAM1,
        CAM2;
    }

    public static enum LimelightPipeline {
        APRILTAG,
        CONE,
        CUBE
    }

    public static NetworkTable getLimelight(CameraName cameraName) {
        return cameraName == CameraName.CAM1 ? limelightLeft : limelightRight;
    }

    public double getHorizontalCrosshairOffsetAngle(CameraName cameraName) {
        return cameraName == CameraName.CAM1 ? limelightLeft.getEntry("tx").getDouble(0)
                : limelightRight.getEntry("tx").getDouble(0);
    }

    public double getVerticalCrosshairOffsetAngle(CameraName cameraName) {
        return cameraName == CameraName.CAM1 ? limelightLeft.getEntry("ty").getDouble(0)
                : limelightRight.getEntry("ty").getDouble(0);
    }

    public boolean hasValidTargets(CameraName cameraName) {
        return cameraName == CameraName.CAM1 ? limelightLeft.getEntry("tv").getDouble(0) > 0
                : limelightRight.getEntry("tv").getDouble(0) > 0;
    }

    public double getTargetArea(CameraName cameraName) {
        return cameraName == CameraName.CAM1 ? limelightLeft.getEntry("ta").getDouble(0)
                : limelightRight.getEntry("ta").getDouble(0);
    }

    public double getSkewArea(CameraName cameraName) {
        return cameraName == CameraName.CAM1 ? limelightLeft.getEntry("ts").getDouble(0)
                : limelightLeft.getEntry("ts").getDouble(0);
    }

    public double getAprilTagID(CameraName cameraName) {
        return cameraName == CameraName.CAM1 ? limelightLeft.getEntry("tid").getDouble(-1)
                : limelightLeft.getEntry("tid").getDouble(-1);
    }

    /**
     * @param limelightMountAngleDegrees how many degrees back is your limelight
     *                                   rotated from
     *                                   perfectly vertical? If the limelight is
     *                                   upside down, be sure to set the orientation
     *                                   to
     *                                   'upside-down' in Limelight finder-tool or
     *                                   local:5801,
     * @param limelightLensHeightInches  distance from the center of the Limelight
     *                                   lens to the floor
     * @param goalHeightInches           distance from the target to the floor
     * @return distance from the limelight to the target in inches
     */
    public double estimateDistance(
            double limelightMountAngleDegrees,
            double limelightLensHeightInches,
            double goalHeightInches, CameraName cameraName) {
        double targetOffsetAngle_Vertical = this.getVerticalCrosshairOffsetAngle(cameraName);
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        // calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
                / Math.tan(angleToGoalRadians);
        // System.out.println(distanceFromLimelightToGoalInches);
        return distanceFromLimelightToGoalInches;
    }

    public LimelightPipeline getPipeline(CameraName cameraName) {
        return LimelightPipeline
                .values()[(int) (cameraName == CameraName.CAM1 ? limelightLeft.getEntry("pipeline").getDouble(0)
                        : limelightRight.getEntry("pipeline").getDouble(0))];
    }

    public void setPipeline(CameraName cameraName, LimelightPipeline pipeline) {
        boolean temp = cameraName == CameraName.CAM1 ? limelightLeft.getEntry("pipeline").setDouble(pipeline.ordinal())
                : limelightRight.getEntry("pipeline").setDouble(pipeline.ordinal());
    }

    @Override
    public void periodic() {
        VALID_TARGETS_PUB_LEFT.set(VALID_TARGETS_SUB_LEFT.get(0.0));
        OFFSET_X_PUB_LEFT.set(OFFSET_X_SUB_LEFT.get(0.0));
        OFFSET_Y_PUB_LEFT.set(OFFSET_Y_SUB_LEFT.get(0.0));
        TARGET_AREA_PUB_LEFT.set(TARGET_AREA_SUB_LEFT.get(0.0));
        SKEW_ROTATION_PUB_LEFT.set(SKEW_ROTATION_SUB_LEFT.get(0.0));

        VALID_TARGETS_PUB_RIGHT.set(VALID_TARGETS_SUB_RIGHT.get(0.0));
        OFFSET_X_PUB_RIGHT.set(OFFSET_X_SUB_RIGHT.get(0.0));
        OFFSET_Y_PUB_RIGHT.set(OFFSET_Y_SUB_RIGHT.get(0.0));
        TARGET_AREA_PUB_RIGHT.set(TARGET_AREA_SUB_RIGHT.get(0.0));
        SKEW_ROTATION_PUB_RIGHT.set(SKEW_ROTATION_SUB_RIGHT.get(0.0));
    }
}